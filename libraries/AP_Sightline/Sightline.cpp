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

extern const AP_HAL::HAL &hal;

// table of user settable parameters
const AP_Param::GroupInfo RangeFinder::var_info[] = {

    // @Param: _FREQUENCY
    // @DisplayName: Sightline frequency
    // @Description: Frequency at which to trigger snapshots
    // @Units: Hz
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("_FREQUENCY", 0, RangeFinder, _scaling[0], 1.0f),

    AP_GROUPEND
};

Sightline::Sightline(AP_SerialManager &_serial_manager) :
    num_instances(0),
    serial_manager(_serial_manager)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  initialise the RangeFinder class. We do detection of attached range
  finders here. For now we won't allow for hot-plugging of
  rangefinders.
 */
void Sightline::init(void)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }

    num_instances = 1;
}

/*
  update RangeFinder state for all instances. This should be called at
  around 10Hz by main loop
 */
void Sightline::update(void)
{

}

void Sightline::handle_msg(mavlink_message_t *msg)
{

}

