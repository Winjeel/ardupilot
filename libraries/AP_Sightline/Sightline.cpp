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

#include "Sightline.h"
#include <AP_BoardConfig/AP_BoardConfig.h>

#include "sightline_protocol.h"

#include <cstring>

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo Sightline::var_info[] = {

    // @Param: _FREQUENCY
    // @DisplayName: Sightline frequency
    // @Description: Frequency at which to trigger snapshots
    // @Units: Hz
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_FREQUENCY", 0, Sightline, _frequency[0], 5.0f),

    AP_GROUPEND
};


#define _DEBUG(s) if (HAL_OS_POSIX_IO) { ::printf(s); }


Sightline::Sightline(AP_SerialManager &_serial_manager) :
    num_instances(0),
    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);

    // TODO: This is the correct way to do it, but it may require a rebuild of
    //       the ground station app. Might need to hardcode the serial port...
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Sightline, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Sightline, 0));
    } else {
        _DEBUG("sightline: no uart!\n");
    }
}

/*
  initialise the Sightline class.
 */
void Sightline::init(void) {
    if (num_instances != 0) {
        // init called a 2nd time?
        _DEBUG("sightline: failed init\n");
        return;
    }

    num_instances = 1;
    init_time = AP_HAL::millis();
    lastDoSnapshotTime = init_time;
    lastSetMetadataTime = init_time;
    lastGetVersionTime = init_time;

    _DEBUG("sightline: init\n");
}


/*
  update Sightline state for all instances. This should be called at
  around 10Hz by main loop
 */
void Sightline::update(void) {
    if (uart == nullptr) {
        _DEBUG("sightline: update no uart\n");
        return;
    }

    // TODO: It looks as though the Discover protocol isn't needed for serial
    //       comms, but if it is, set it up here...

    size_t numBytes = uart->available();
    size_t bufferFree = msgBuffer.getBytesFree();
    if (numBytes > bufferFree) {
        numBytes = bufferFree;
    }

    while (numBytes-- > 0) {
        msgBuffer.push(uart->read());
    }

    SL_MsgId msgType = msgBuffer.assess();
    while (msgType != SL_Msg_None) {
        switch (msgType) {
        case SL_Msg_VersionNumber: {
            SL_Cmd_VersionNumber versionMsg;

            _DEBUG("sightline: got SL_Cmd_VersionNumber:\n");
            msgBuffer.copyData(&versionMsg, sizeof(versionMsg)); // TODO: check length is correct
            if (HAL_OS_POSIX_IO) { ::printf("           talking to hwType=%d:\n", versionMsg.hwType); }

            break;
        }

        // TODO: Handle other received messages
        default:
            if (HAL_OS_POSIX_IO) { ::printf("sightline: got message 0x%02x\n", msgType); }
            break;
        }

        msgType = msgBuffer.assess();
    }


    // send periodic messages
    const uint32_t kDoSnapshotPeriod = (uint32_t)(_frequency[0] * 1000); // Hz to millis
    const uint32_t kSetMetadataPeriod = 5000; // millis
    const uint32_t kGetVersionPeriod = 5000; // millis

    uint32_t timeNow = AP_HAL::millis();

    if (timeNow > (lastGetVersionTime + kGetVersionPeriod)) {
        uint8_t msg[6] = { SL_MAGIC_1, SL_MAGIC_2, 3, SL_Msg_GetParameters, SL_Msg_GetVersionNumber, 0x73, }; // TODO: use proper struct
        uart->write((const uint8_t *)&msg, sizeof(msg));
        lastGetVersionTime = timeNow;
    }
    if (timeNow > (lastDoSnapshotTime + kDoSnapshotPeriod)) {
        SL_Cmd_SetMetadataValues msg = {0}; // TODO: Populate...
        uart->write((const uint8_t *)&msg, sizeof(msg));
        lastDoSnapshotTime = timeNow;
    }
    if (timeNow > (lastSetMetadataTime + kSetMetadataPeriod)) {
        SL_Cmd_DoSnapshot msg = {0}; // TODO: Populate...
        uart->write((const uint8_t *)&msg, sizeof(msg));
        lastSetMetadataTime = timeNow;
    }
}

void Sightline::handle_msg(mavlink_message_t *msg) {
    // TODO
}

