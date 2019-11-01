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

#include "autopilot-interface.h"

#include "SoftwareVersion.h"
#include "HardwareVersion.h"
#include "DiagnosticMessage.h"

#include "OpticalFlowState.h"
#include "AdcState.h"

// #include <cstring>

#include "GCS_MAVLink/GCS.h"


extern const AP_HAL::HAL &hal;


AP_PpdsMotorPod *AP_PpdsMotorPod::_singleton = nullptr;

AP_PpdsMotorPod::AP_PpdsMotorPod(void) :
            uart(nullptr),
            is_connected(false),
            driver_init_time(AP_HAL::millis())
{
    _singleton = this;
#if (CORVO_DEBUG)
    debug_tick = 0;
#endif
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

#if (CORVO_DEBUG)
    #define _debug_freq(freq, lvl, fmt, ...)                \
        do {                                                  \
            if ((debug_tick % freq) == 0) {                   \
                gcs().send_text(lvl, fmt, ##__VA_ARGS__);     \
            }                                                 \
        } while (0)
#else
    #define _debug_freq(freq, lvl, fmt, ...)
#endif
#define _debug(lvl, fmt, ...)        _debug_freq( 1, lvl, fmt, ##__VA_ARGS__)
#define _debug_sample(lvl, fmt, ...) _debug_freq(10, lvl, fmt, ##__VA_ARGS__)


void AP_PpdsMotorPod::_handleRequest(void) {
    switch (msg_buffer.getID()) {
        default: {
            // ignore others for now
            break;
        }
    }
}

void AP_PpdsMotorPod::_handleMsg(size_t const kUartNumBytes) {
    CorvoMsgID const kMsg_id = msg_buffer.getID();

    if ((interface_state == IF_State::Waiting) && (kMsg_id != INTERFACE_VERSION)) {
        // while we don't know what we're talking to, the only message we will
        // process is an INTERFACE_VERSION message.
        msg_buffer.consume();
        return;
    }

    switch (kMsg_id) {
        case SOFTWARE_VERSION: {
            uint8_t tmp[getSoftwareVersionMaxDataLength()];
            msg_buffer.copyData(tmp, sizeof(tmp));

            SoftwareVersion_t sw_version;
            int idx = 0;
            decodeSoftwareVersion_t(tmp, &idx, &sw_version);

            char build_char = '?';
            if (sw_version.build_type == CV_BLD_DEVELOPMENT) { build_char = 'D'; }
            if (sw_version.build_type == CV_BLD_INTEGRATION) { build_char = 'I'; }
            if (sw_version.build_type == CV_BLD_PRODUCTION)  { build_char = 'P'; }
            gcs().send_text(MAV_SEVERITY_INFO,
                            "MotorPod SW: %.12s v%u.%u.%u-%c hash=%lx",
                            sw_version.id,
                            sw_version.major, sw_version.minor, sw_version.patch, build_char,
                            sw_version.git_hash);

            break;
        }

        case HARDWARE_VERSION: {
            uint8_t tmp[getHardwareVersionMaxDataLength()];
            msg_buffer.copyData(tmp, sizeof(tmp));

            HardwareVersion_t hw_version;
            int idx = 0;
            decodeHardwareVersion_t(tmp, &idx, &hw_version);

            gcs().send_text(MAV_SEVERITY_INFO,
                            "MotorPod HW: %.12s rev%c.%u",
                            hw_version.id,
                            hw_version.major, hw_version.minor);

            break;
        }

        case INTERFACE_VERSION: {
            uint8_t tmp[getInterfaceVersionMaxDataLength()];
            msg_buffer.copyData(tmp, sizeof(tmp));

            InterfaceVersion_t if_version;
            int idx = 0;
            decodeInterfaceVersion_t(tmp, &idx, &if_version);

            if_version_receive_ms = AP_HAL::millis();
            _assessInterfaceVersion(if_version);

            gcs().send_text(MAV_SEVERITY_INFO,
                            "MotorPod IF: %.12s v%u.%u.%u",
                            if_version.id,
                            if_version.major, if_version.minor, if_version.patch);

            break;
        }

        case DIAGNOSTIC_MSG: {
            uint8_t tmp[getDiagnosticMessageMaxDataLength()];
            msg_buffer.copyData(tmp, sizeof(tmp));

            DiagnosticMessage_t msg;
            int idx = 0;
            decodeDiagnosticMessage_t(tmp, &idx, &msg);

            enum MAV_SEVERITY kSeverityMap[] = {
                MAV_SEVERITY_CRITICAL, // CV_LVL_FATAL
                MAV_SEVERITY_ERROR,    // CV_LVL_ERROR
                MAV_SEVERITY_WARNING,  // CV_LVL_WARN
                MAV_SEVERITY_INFO,     // CV_LVL_INFO
                MAV_SEVERITY_DEBUG,    // CV_LVL_DEBUG
            };
            if (msg.level > CV_LVL_DEBUG) {
                msg.level = CV_LVL_DEBUG;
            }

            gcs().send_text(kSeverityMap[msg.level], "MotorPod Msg: %.*s", sizeof(msg.str), msg.str);

            break;
        }

        case OPTICAL_FLOW_STATE: {
            WITH_SEMAPHORE(_flow_sem);

            _debug_sample(MAV_SEVERITY_DEBUG, "\tMotorPod: got OpticalFlow msg 0x%02x (sz: %u)", msg_id, dataSz);

            uint8_t tmp[getOpticalFlowStateMaxDataLength()];
            msg_buffer.copyData(tmp, sizeof(tmp));

            OpticalFlowState_t flow;
            int idx = 0;
            decodeOpticalFlowState_t(tmp, &idx, &flow);

            _debug_sample(MAV_SEVERITY_DEBUG, "\t\tsequence=%u", flow.sequence);
            _debug_sample(MAV_SEVERITY_DEBUG, "\t\ttimeDelta=%.3fms", flow.timeDelta_us / 1000.0);
            _debug_sample(MAV_SEVERITY_DEBUG, "\t\tisMoving=%u", flow.isMoving);
            _debug_sample(MAV_SEVERITY_DEBUG, "\t\tsurfaceQual=%u", flow.surfaceQuality);
            _debug_sample(MAV_SEVERITY_DEBUG, "\t\tflowDelta.X=%d", flow.flowDelta.x);
            _debug_sample(MAV_SEVERITY_DEBUG, "\t\tflowDelta.Y=%d", flow.flowDelta.y);

            _flow_data.receiveTime_us = uart->receive_time_constraint_us(kUartNumBytes);
            _flow_data.surfaceQuality = flow.surfaceQuality;
            _flow_data.delta.x += flow.flowDelta.x;
            _flow_data.delta.y += flow.flowDelta.y;
            _flow_data.delta.t_us += flow.timeDelta_us;

            break;
        }

        case ADC_STATE: {
            WITH_SEMAPHORE(_adc_sem);

            _debug_sample(MAV_SEVERITY_DEBUG, "\tMotorPod: got ADC msg 0x%02x (sz: %u)", msg_id, dataSz);

            uint8_t tmp[getAdcStateMaxDataLength()];
            msg_buffer.copyData(tmp, sizeof(tmp));

            AdcState_t adc;
            int idx = 0;
            decodeAdcState_t(tmp, &idx, &adc);

            if (_adc_data.deltaT_us == 0 || adc.timeDelta_us == 0) {
                // use instantaneous values
                _adc_data.current = adc.current;
                _adc_data.voltage = adc.voltage;
                _adc_data.temperature = adc.temperature;
            } else {
                // calculate running average
                _adc_data.current = ((adc.current * adc.timeDelta_us) + (_adc_data.current * _adc_data.deltaT_us)) /
                                        (adc.timeDelta_us + _adc_data.deltaT_us);
                _adc_data.voltage = ((adc.voltage * adc.timeDelta_us) + (_adc_data.voltage * _adc_data.deltaT_us)) /
                                        (adc.timeDelta_us + _adc_data.deltaT_us);
                _adc_data.temperature = ((adc.temperature * adc.timeDelta_us) + (_adc_data.temperature * _adc_data.deltaT_us)) /
                                            (adc.timeDelta_us + _adc_data.deltaT_us);
            }

            _adc_data.deltaT_us += adc.timeDelta_us;

            _debug_sample(MAV_SEVERITY_DEBUG, "\t\tsequence=%u", adc.sequence);
            _debug_sample(MAV_SEVERITY_DEBUG, "\t\ttimeDelta=%.3fms", adc.timeDelta_us / 1000.0);
            _debug_sample(MAV_SEVERITY_DEBUG, "\t\tcurrent=%f", adc.current);
            _debug_sample(MAV_SEVERITY_DEBUG, "\t\tvoltage=%f", adc.voltage);
            _debug_sample(MAV_SEVERITY_DEBUG, "\t\ttemp=%f", adc.temperature);

            break;
        }

        // TODO: Handle other received messages
        default: {
            _debug_sample(MAV_SEVERITY_DEBUG, "\tMotorPod:   got msg 0x%02x", msg_id);
            break;
        }
    }

    // Done with the current msg, so remove it from the buffer (otherwise
    // we'll get an infinite loop)
    size_t eaten = msg_buffer.consume();
    _debug_sample(MAV_SEVERITY_DEBUG, "\tMotorPod: ate %lu bytes", eaten);
    (void)eaten;
}


void AP_PpdsMotorPod::_requestPacket(PacketId pktId) {
    CorvoPacket pkt = { 0, 0, 0, nullptr, 0 };
    finishPpdsMotorPodPacket(&pkt, 0, pktId);
    _sendCorvoPacket(&pkt);
}


void AP_PpdsMotorPod::_assessInterfaceVersion(InterfaceVersion_t const &kIF_version) {
    if (if_version_request_ms >= if_version_receive_ms) {
        interface_state = IF_State::Waiting;
    } else {
        // TODO: Work out a way to populate/generate this...
        //       For now, use the ProtoGen version string
        uint8_t pg_version[6] = getPpdsMotorPodVersion();

        InterfaceVersion_t const kSupportedVersion = {
            .id = "F01-TBD-TBD", // ICD, Motor Controller->Autopilot
            // TODO: This is very fragile, but works for now...
            .major = (uint8_t)(pg_version[0] - '0'),
            .minor = (uint8_t)(pg_version[2] - '0'),
            .patch = (uint8_t)(pg_version[4] - '0'),
        };

        if (   strncmp(kIF_version.id, kSupportedVersion.id, sizeof(kIF_version.id)) == 0
            && kIF_version.major == kSupportedVersion.major
            && kIF_version.minor == kSupportedVersion.minor
            && kIF_version.patch == kSupportedVersion.patch) {
            interface_state = IF_State::Ok;
        } else {
            gcs().send_text(MAV_SEVERITY_ERROR, "MotorPod: Bad Interface Version!");
            interface_state = IF_State::Mismatch;
        }
    }
}

/*
  update AP_PpdsMotorPod state for all instances. This should be called at
  around 10Hz by main loop
 */
void AP_PpdsMotorPod::update(void) {
    _debug_sample(MAV_SEVERITY_WARNING, "[%8lums] MotorPod: update", AP_HAL::millis());

    // TODO: re-init?
    if (uart == nullptr) {
        _debug(MAV_SEVERITY_ERROR, "\tMotorPod: uart == null");
        return;
    }

    if (!uart->is_initialized()) {
        // TODO: is this needed, and how do we recover?
        _debug(MAV_SEVERITY_ERROR, "\tMotorPod: uart !init");
        return;
    }

    if (interface_state == IF_State::Mismatch) {
        _debug(MAV_SEVERITY_ERROR, "\tMotorPod: bad IF version!");
        return;
    }

    if (interface_state == IF_State::Waiting) {
        uint32_t now = AP_HAL::millis();
        bool first_time = (if_version_request_ms == 0);
        bool waited_too_long = ((now - if_version_request_ms) > 1000);
        if (first_time || waited_too_long) {
            if_version_request_ms = now;
            _requestPacket(INTERFACE_VERSION);
            _requestPacket(SOFTWARE_VERSION);
            _requestPacket(HARDWARE_VERSION);
        }
    }

    size_t const kUartNumBytes = uart->available();
    size_t num_bytes = kUartNumBytes;
    size_t buffer_free = msg_buffer.getBytesFree();
    if (num_bytes > buffer_free) {
        num_bytes = buffer_free;
    }

    if (num_bytes) {
        _debug_sample(MAV_SEVERITY_DEBUG, "\tMotorPod: got %lu bytes", num_bytes);

        while (num_bytes-- > 0) {
            int16_t c = uart->read();
            if (c >= 0) {
                msg_buffer.push(c);
            }
        }

        while (msg_buffer.hasMsg()) {
            uint8_t const isRequest = (msg_buffer.getDataSz() == 0);
            if (isRequest) {
                _handleRequest();
            }else {
                _handleMsg(kUartNumBytes);
            }

        }
    } else {
        _debug(MAV_SEVERITY_ERROR, "\tMotorPod: got no bytes", num_bytes);
    }

#if (CORVO_DEBUG)
    // update debug tick
    debug_tick++;
#endif // (CORVO_DEBUG)
}


uint16_t AP_PpdsMotorPod::_sendCorvoPacket(CorvoPacket const * const pkt) {
    uint16_t sentBytes = 0;

    size_t toSend = 3;
    size_t tx = uart->write((uint8_t *)pkt, toSend);
    if ((pkt->dataSz > 0) && (tx == toSend)) {
        sentBytes += tx;
        toSend = pkt->dataSz;
        tx = uart->write(pkt->data, toSend);
    }
    if (tx == toSend) {
        sentBytes += tx;
        sentBytes += uart->write(&pkt->crc, 1);
    }

    return sentBytes;
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

bool AP_PpdsMotorPod::getAdcData(AdcData * const adc_data) {
    WITH_SEMAPHORE(_adc_sem);
    bool has_data = (_adc_data.deltaT_us > 0);

    if (has_data) {
        memcpy(adc_data, &_adc_data, sizeof(*adc_data));
    }

    return has_data;
}

void AP_PpdsMotorPod::clearAdcData(void) {
    WITH_SEMAPHORE(_adc_sem);
    _adc_data.deltaT_us = 0;
}

namespace AP {
    AP_PpdsMotorPod *motorPod() {
        return AP_PpdsMotorPod::get_singleton();
    }
}
