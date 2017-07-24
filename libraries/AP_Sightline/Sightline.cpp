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
#include <GCS_Mavlink/GCS.h>

#include "sightline_protocol.h"

#include <cstring>

// test JIRA/Bitbucket integration...

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo Sightline::var_info[] = {

    // @Param: _FREQUENCY
    // @DisplayName: Sightline frequency
    // @Description: Frequency at which to trigger snapshots
    // @Units: Hz
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FREQUENCY", 0, Sightline, frequency, 5.0f),

    AP_GROUPEND
};


Sightline::Sightline(AP_SerialManager &_serial_manager) :
    serial_manager(_serial_manager),
    uart(nullptr),

    init_time(AP_HAL::millis()),
    nextDoSnapshotTime(-1),
    nextSetMetadataTime(-1),
    nextGetVersionTime(-1)
{
    AP_Param::setup_object_defaults(this, var_info);

    if (SIGHTLINE_DEBUG) {
        tick = 0;
    }
}

/*
  initialise the Sightline class.
 */
void Sightline::init(void) {

    //hal.console->printf("sightline: init\n");

    // TODO: This is the correct way to do it, but it may require a rebuild of
    //       the ground station app. Might need to hardcode the serial port...
    if (uart == nullptr) {
        uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Sightline, 0);

        if (uart != nullptr) {
            //hal.console->printf("sightline: init found uart\n");
            uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Sightline, 0));
        } else {
            //hal.console->printf("sightline: init no uart\n");
        }
    }

    nextDoSnapshotTime = init_time;
    nextSetMetadataTime = init_time;
    nextGetVersionTime = init_time;
}

/*
  update Sightline state for all instances. This should be called at
  around 10Hz by main loop
 */
void Sightline::update(void) {

    uint32_t timeNow = AP_HAL::millis();

    if (SIGHTLINE_DEBUG) {
        tick = ((tick + 1) % 10);
        if (tick != 0) {
            return;
        }
        gcs().send_text(MAV_SEVERITY_DEBUG, "[%lums] SightLine update", timeNow);
    }

    if (uart == nullptr) {
        gcs().send_text(MAV_SEVERITY_WARNING, "    SL: err no uart\n");
        return;
    }

    // TODO: It looks as though the Discover protocol isn't needed for serial
    //       comms, but if it is, set it up here...

    size_t numBytes = uart->available();
    size_t bufferFree = msgBuffer.getBytesFree();
    if (numBytes > bufferFree) {
        numBytes = bufferFree;
    }

    if (SIGHTLINE_DEBUG && numBytes) {
        gcs().send_text(MAV_SEVERITY_DEBUG, "    SL: got %lu bytes", numBytes);
    }

    while (numBytes-- > 0) {
        msgBuffer.push(uart->read());
    }

    SL_MsgId msgType = msgBuffer.assess();
    while (msgType != SL_MsgId_None) {
        switch (msgType) {
        case SL_MsgId_VersionNumber: {
            SL_Cmd_VersionNumber versionMsg;

            gcs().send_text(MAV_SEVERITY_DEBUG, "    SL: got msg SL_Cmd_VersionNumber\n");
            msgBuffer.copyData(&versionMsg, sizeof(versionMsg)); // TODO: check length is correct
            //hal.console->printf("           talking to hwType=%d:\n", versionMsg.hwType);
            gcs().send_text(MAV_SEVERITY_DEBUG, "    SL:     talking to hwType=%d:\n", versionMsg.hwType);

            break;
        }

        // TODO: Handle other received messages
        default:
            gcs().send_text(MAV_SEVERITY_DEBUG, "    SL: got msg 0x%02x", msgType);
            break;
        }

        msgType = msgBuffer.assess();
    }


    // send periodic messages
    const uint32_t kDoSnapshotPeriod = static_cast<uint32_t>(frequency.get() * 1000); // Hz to millis
    const uint32_t kSetMetadataPeriod = 5000; // millis
    const uint32_t kGetVersionPeriod = 5000; // millis

    SL_MsgHeader header = {
            .magic1 = SL_MAGIC_1,
            .magic2 = SL_MAGIC_2,
    };

    if (timeNow > nextGetVersionTime) {
        uint8_t msg[6] = { SL_MAGIC_1, SL_MAGIC_2, 3, SL_MsgId_GetParameters, SL_MsgId_GetVersionNumber, 0x73, }; // TODO: use proper struct
        uart->write((const uint8_t *)&msg, sizeof(msg));
        nextGetVersionTime = timeNow + kGetVersionPeriod;
        gcs().send_text(MAV_SEVERITY_DEBUG, "    SL: sent msg 0x%02x", SL_MsgId_GetParameters);
    }
    if (timeNow > nextSetMetadataTime) {
        // TODO: Populate fully...
        SL_Cmd_SetMetadataValues msg = {
                .updateMask = 0x0000,
                .utcTime_us = (uint64_t)timeNow * 1000, // millis to micros
        };
        header.length = sizeof(SL_Cmd_SetMetadataValues) + 2;
        header.id = SL_MsgId_SetMetadataValues;

        const uint8_t crc = msgBuffer.calculateCrc(
                SL_MsgId_SetMetadataValues, (uint8_t const * const)&msg, sizeof(msg));

        uart->write((const uint8_t *)&header, sizeof(header));
        uart->write((const uint8_t *)&msg, sizeof(msg));
        uart->write(&crc, sizeof(crc));

        nextSetMetadataTime = timeNow + kSetMetadataPeriod;
        gcs().send_text(MAV_SEVERITY_DEBUG, "    SL: sent msg 0x%02x", header.id);
    }
    if (timeNow > nextDoSnapshotTime) {
        // TODO: Populate properly...
        SL_Cmd_DoSnapshot msg = {
                .frameStep = 1,
                .numSnapshots = 1,
                .filenameLen = 5, // max 64
                .baseFilename = { 'c', 'o', 'r', 'v', 'o' },
                .maskSnapAllCameras = 0xFF,
        };
        header.length = sizeof(SL_Cmd_DoSnapshot) + 2;
        header.id = SL_MsgId_DoSnapshot;

        const uint8_t crc = msgBuffer.calculateCrc(
                SL_MsgId_DoSnapshot, (uint8_t const * const)&msg, sizeof(msg));

        uart->write((const uint8_t *)&header, sizeof(header));
        uart->write((const uint8_t *)&msg, sizeof(msg));
        uart->write(&crc, sizeof(crc));

        nextDoSnapshotTime = timeNow + kDoSnapshotPeriod;
        gcs().send_text(MAV_SEVERITY_DEBUG, "    SL: sent msg 0x%02x", header.id);
    }
}

void Sightline::handle_msg(mavlink_message_t *msg) {
    // TODO
}

