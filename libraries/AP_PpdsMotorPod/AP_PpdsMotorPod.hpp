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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "CorvoMsgBuffer.hpp"
#include "InterfaceVersion.h"


#define CORVO_DEBUG (false)


class AP_PpdsMotorPod
{
public:

    AP_PpdsMotorPod(void);

    // detect and initialise any available SightLine boards
    void init(AP_SerialManager &_serial_manager);

    // update state of all SightLine boards. Should be called at
    // around 10Hz from main loop
    void update(void);


    typedef struct {
        uint32_t receiveTime_us;
        uint8_t surfaceQuality;
        struct {
            uint32_t t_us;
            int32_t x;
            int32_t y;
        } delta;
    } FlowData;

    typedef struct {
        float current;
        float voltage;
        float temperature;
        uint32_t deltaT_us;
    } AdcData;

    // Copy the accumulated flow data.
    // Returns true if data is available and has been copied.
    bool getFlowData(FlowData * const flow_data);

    // Clear the accumulated flow data
    void clearFlowData(void);

    bool getAdcData(AdcData * const adc_data);
    void clearAdcData(void);

    // get singleton instance
    static AP_PpdsMotorPod *get_singleton() {
        return AP_PpdsMotorPod::_singleton;
    }

private:
    enum IF_State {
        Waiting,
        Ok,
        Mismatch,
    };

    void _handleMsg(size_t const kUartNumBytes);
    void _handleRequest(void);
    uint16_t _sendCorvoPacket(CorvoPacket const * const pkt);
    void _requestPacket(PacketId pktId);

    void _assessInterfaceVersion(InterfaceVersion_t const &kIF_version);
    enum IF_State interface_state = IF_State::Waiting;
    uint32_t if_version_request_ms = 0;
    uint32_t if_version_receive_ms = 0;

    AP_HAL::UARTDriver *uart = nullptr;

    CorvoMsgBuffer msg_buffer;

    bool is_connected = false;

    uint32_t driver_init_time = 0;

    FlowData _flow_data = {};
    HAL_Semaphore _flow_sem;

    AdcData _adc_data = {};
    HAL_Semaphore _adc_sem;

    static AP_PpdsMotorPod *_singleton;

#if (CORVO_DEBUG)
    uint8_t debug_tick = 0;
#endif // CORVO_DEBUG
};

namespace AP {
    AP_PpdsMotorPod *motorPod();
}
