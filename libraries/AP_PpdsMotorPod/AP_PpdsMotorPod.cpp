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


#include <cstring>

extern const AP_HAL::HAL &hal;


AP_PpdsMotorPod::AP_PpdsMotorPod(void) :
            uart(nullptr),

            is_connected(false),

            driver_init_time(AP_HAL::millis())
{
    if (CORVO_DEBUG) {
        tick = 0;
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

/*
  update AP_PpdsMotorPod state for all instances. This should be called at
  around 10Hz by main loop
 */
void AP_PpdsMotorPod::update(void) {

    uint32_t time_now = AP_HAL::millis();

    if (CORVO_DEBUG) {
        tick = ((tick + 1) % 10);
        if (tick == 0) {
            gcs().send_text(MAV_SEVERITY_WARNING, "[%lums] PMP: update", time_now);
        }
    }

    // TODO: re-init?
    if (uart == nullptr) {
        return;
    }

    if (!uart->is_initialized()) {
        // TODO: is this needed, and how do we recover?
        gcs().send_text(MAV_SEVERITY_WARNING, "    PMP: uart !init");
        return;
    }

    size_t num_bytes = uart->available();
    size_t buffer_free = msg_buffer.getBytesFree();
    if (num_bytes > buffer_free) {
        num_bytes = buffer_free;
    }

    if (CORVO_DEBUG && num_bytes) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "    PMP: got %lu bytes", num_bytes);
    }

    while (num_bytes-- > 0) {
        msg_buffer.push(uart->read());
    }

    while (msg_buffer.hasMsg()) {
        CorvoMsgID msg_id = msg_buffer.getID();
        switch (msg_id) {


            // TODO: Handle other received messages
            default: {
                gcs().send_text(MAV_SEVERITY_DEBUG, "    PMP: got msg 0x%02x", msg_id);
                break;
            }
        }

        // Done with the current msg, so remove it from the buffer (otherwise
        // we'll get an infinite loop)
        size_t eaten = msg_buffer.consume();
        if (CORVO_DEBUG) {
            gcs().send_text(MAV_SEVERITY_DEBUG, "    PMP:     consumed=%lu:", eaten);
        } else {
            (void)eaten; // suppress unused warning
        }
    }

    if (!is_connected) {
        // TODO: send query message
    } else {
        // TODO: send update request(?)
    }
}
