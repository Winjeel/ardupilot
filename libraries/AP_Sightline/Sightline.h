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
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

// Maximum number of range finder instances available on this platform
#define SIGHTLINE_MAX_INSTANCES 1

 
class Sightline
{
public:

    Sightline(AP_SerialManager &_serial_manager);

    // parameters for each instance
    AP_Float _frequency[SIGHTLINE_MAX_INSTANCES];

    static const struct AP_Param::GroupInfo var_info[];
    
    // Return the number of range finder instances
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    // detect and initialise any available SightLine boards
    void init(void);

    // update state of all SightLine boards. Should be called at
    // around 10Hz from main loop
    void update(void);

    // TODO: Handle an incoming MAVLink message
    void handle_msg(mavlink_message_t *msg);

private:
    uint8_t num_instances;
    AP_SerialManager &serial_manager;
};
