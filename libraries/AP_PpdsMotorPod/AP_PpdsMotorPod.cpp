/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_PpdsMotorPod.hpp"

#include "GCS_MAVLink/GCS.h"

#include "autopilot-interface.h"
#include "OpticalFlowState.h"

#include <cstring>

extern const AP_HAL::HAL &hal;


AP_PpdsMotorPod *AP_PpdsMotorPod::_singleton = nullptr;

AP_PpdsMotorPod::AP_PpdsMotorPod(void) :
            uart(nullptr),
            is_connected(false),
            driver_init_time(AP_HAL::millis())
{
    _singleton = this;
    if (CORVO_DEBUG) {
        debug_tick = 0;
    }
}


/*
  initialise the AP_PpdsMotorPod class.
 */
void AP_PpdsMotorPod::init(AP_SerialManager &_serial_manager) {
    if (uart == nullptr) {
        uart = _serial_manager.find_serial(AP_SerialManager::SerialProtocol_Corvo, 0);

        if (uart != nullptr) {
            uart->begin(_serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Corvo, 0));
        }
    }
}

#define _debug_freq(freq, lvl, fmt, ...)                  \
    do {                                                  \
        if (CORVO_DEBUG && ((debug_tick % freq) == 0)) {  \
            gcs().send_text(lvl, fmt, ##__VA_ARGS__);     \
        }                                                 \
    } while (0)
#define _debug(lvl, fmt, ...) _debug_freq(10, lvl, fmt, ##__VA_ARGS__)

/*
  update AP_PpdsMotorPod state for all instances. This should be called at
  around 10Hz by main loop
 */
void AP_PpdsMotorPod::update(void) {
    WITH_SEMAPHORE(_flow_sem);

    uint32_t time_now = AP_HAL::millis();

    _debug(MAV_SEVERITY_WARNING, "[%8lums] PMP: update", time_now);

    // TODO: re-init?
    if (uart == nullptr) {
        _debug(MAV_SEVERITY_WARNING, "\tPMP: uart == null");
        return;
    }

    if (!uart->is_initialized()) {
        // TODO: is this needed, and how do we recover?
        _debug(MAV_SEVERITY_WARNING, "\tPMP: uart !init");
        return;
    }

    size_t num_bytes = uart->available();
    size_t buffer_free = msg_buffer.getBytesFree();
    if (num_bytes > buffer_free) {
        num_bytes = buffer_free;
    }

    if (num_bytes) {
        _debug(MAV_SEVERITY_DEBUG, "\tPMP: got %lu bytes", num_bytes);

        while (num_bytes-- > 0) {
            msg_buffer.push(uart->read());
        }

        while (msg_buffer.hasMsg()) {
            CorvoMsgID msg_id = msg_buffer.getID();
            uint8_t dataSz = msg_buffer.getDataSz();
            switch (msg_id) {
                case OPTICAL_FLOW_STATE: {

                    _debug(MAV_SEVERITY_DEBUG, "\tPMP: got OpticalFlow msg 0x%02x (sz: %u)", msg_id, dataSz);
                    if (dataSz >= getOpticalFlowStateMinDataLength()) {
                        uint8_t tmp[getOpticalFlowStateMaxDataLength()];
                        int sz = msg_buffer.copyData(tmp, sizeof(tmp));
                        (void)sz;

                        OpticalFlowState_t flow;
                        int idx = 0;
                        decodeOpticalFlowState_t(tmp, &idx, &flow);

                        _debug(MAV_SEVERITY_DEBUG, "\t\ttimeDelta_us=%lums", flow.timeDelta_us);
                        _debug(MAV_SEVERITY_DEBUG, "\t\tsurfaceQual=%u", flow.surfaceQuality);
                        _debug(MAV_SEVERITY_DEBUG, "\t\tflowDelta.X=%f", flow.flowDelta.x);
                        _debug(MAV_SEVERITY_DEBUG, "\t\tflowDelta.Y=%f", flow.flowDelta.y);

                        _flow_data.surfaceQuality = flow.surfaceQuality;
                        _flow_data.delta.x += flow.flowDelta.x;
                        _flow_data.delta.y += flow.flowDelta.y;
                        _flow_data.delta.t_us += flow.timeDelta_us;
                    }

                    break;
                }

                // TODO: Handle other received messages
                default: {
                    _debug(MAV_SEVERITY_DEBUG, "\tPMP:   got msg 0x%02x", msg_id);
                    break;
                }
            }

            // Done with the current msg, so remove it from the buffer (otherwise
            // we'll get an infinite loop)
            size_t eaten = msg_buffer.consume();
            _debug(MAV_SEVERITY_DEBUG, "\tPMP: ate %lu bytes", eaten);
        }
    } else {
        _debug(MAV_SEVERITY_DEBUG, "\tPMP: got no bytes", num_bytes);
    }

#define SEND_PACKET (0)
#if SEND_PACKET
    CorvoPacket pkt{ 0, 0, 0, nullptr, 0, };
    finishPpdsMotorPodPacket(&pkt, 0, OPTICAL_FLOW_STATE);

    uint16_t sentBytes = 0;

    size_t toSend = 3;
    size_t tx = uart->write((uint8_t *)&pkt, toSend);
    if (tx == toSend) {
        sentBytes += tx;
        toSend = pkt.dataSz;
        tx = uart->write(pkt.data, toSend);
    }
    if (tx == toSend) {
        sentBytes += tx;
        sentBytes += uart->write(&pkt.crc, 1);
    }
    _debug(MAV_SEVERITY_DEBUG, "\tPMP: sent %lu bytes", sentBytes);
    if (sentBytes == 4) {
        _debug(MAV_SEVERITY_DEBUG, "\t\t{ %u, %u, %u, %u }", pkt.sentinel, pkt.id, pkt.dataSz, pkt.crc);
    }
#endif // SEND_PACKET


    if (!is_connected) {
        // TODO: send query message
    } else {
        // TODO: send update request(?)
    }

    // update debug tick
    debug_tick++;
}

bool AP_PpdsMotorPod::getFlowData(FlowData * const flow_data) {
    WITH_SEMAPHORE(_flow_sem);
    bool has_data = (_flow_data.delta.t_us > 0);

    if (has_data) {
        memcpy(flow_data, &_flow_data, sizeof(*flow_data));
    }

    return has_data;
}

void AP_PpdsMotorPod::clearFlowData(void) {
    WITH_SEMAPHORE(_flow_sem);
    memset(&_flow_data, 0, sizeof(_flow_data));
}


namespace AP {
    AP_PpdsMotorPod *motorPod() {
        return AP_PpdsMotorPod::get_singleton();
    }
}
