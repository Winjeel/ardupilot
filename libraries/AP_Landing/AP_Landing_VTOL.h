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

#include <AP_Param/AP_Param.h>
#include <AP_Mission/AP_Mission.h>
#include <AP_Common/AP_Common.h>
#include <AP_SpdHgtControl/AP_SpdHgtControl.h>
#include <AP_Navigation/AP_Navigation.h>
#include <GCS_MAVLink/GCS.h>
#include <PID/PID.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;
class AP_Landing;

/// @class  AP_Landing_VTOL
/// @brief  Class managing Plane wind adaptive VTOL landing entry methods
class AP_Landing_VTOL
{
private:
    friend class AP_Landing;

    // constructor
    AP_Landing_VTOL(AP_Landing &_landing) :
        landing(_landing)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    AP_Landing &landing;

    static const struct AP_Param::GroupInfo var_info[];

    // wind adaptive VTOL landing entry members
    enum vtol_stage {
        VTOL_STAGE_FLY_TO_LANDING,    // Fly to the landing point
        VTOL_STAGE_HEIGHT_CAPTURE,    // Loiter until we have reached the required height
        VTOL_STAGE_WAIT_FOR_BREAKOUT, // Wait until the aircraft is aligned for the optimal breakout and has turned sufficiently to have a valid wind estimate.
        VTOL_STAGE_LAND_VTOL,         // Commence VTOL transition over the landing_point
    };

    AP_Float _vtol_decel;        // average deceleration rate during VTOL transition used to calculate predicted_decel_distance (m)
    AP_Float _vtol_xwind_max;    // maximum allowed cross wind for VTOL transition (m/s)
    AP_Float _vtol_twind_max;    // maximum allowed tail wind for VTOL transition (m/s)
    int32_t _loiter_sum_cd;      // used for tracking the progress on loitering (centi-deg)
    vtol_stage _stage;
    Location _landing_point;     // The location we want to land on
    Location _loiter_point;      // centre of an offset loiter that positions the vehicle for an into wind recovery over the landing_point
    Location _entry_point;       // The location when starting the landing
    Location _extended_approach; // waypoint 1000m beyond the landing_point
    float _target_heading_deg;   // target heading for the VTOL transition (degrees)
    int32_t _last_target_bearing;   // used for tracking the progress on loitering (centi-degrees)
    float _predicted_decel_distance = 0.0f; // distance the aircraft is predicted to travel during VTOL transition (m)
    float _approach_alt_offset;     // approach altitude offset used to handle absolute waypoint height difference (m)
    bool _low_wind_overshoot;        // true if we have tried to use a low wind straight in approach and have overshot
    bool _use_loiter;            // true if wind conditions require a circular loiter with into wind recovery
    bool _use_direct;            // true if wind conditions require a direct approach
    Vector2f _approach_vec_unit; // NE vector from landing entry to desired touchdown point
    int16_t _timeout_count;      // Counter used to determine if a mode transtion is taking too long
    bool _commanded_go_around;   // true when a go around and continuation in loiter for one more turn has been requested.
    float _loiter_radius;       // radius in metres used for the recovery loiter pattern - adjusts to accomodate wind

    //public AP_Landing interface
    void do_land(const AP_Mission::Mission_Command& cmd, const float relative_altitude);
    void verify_abort_landing(const Location &prev_WP_loc, Location &next_WP_loc, bool &throttle_suppressed);
    bool verify_land(const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
            const float height, const float sink_rate, const float wp_proportion, const uint32_t last_flying_ms,
            const bool is_armed, const bool is_flying, const bool rangefinder_state_in_range);
    bool request_go_around(void);
    bool get_target_altitude_location(Location &location);
    int32_t get_target_airspeed_cm(void) const;
    bool is_flying_forward(void) const;
    bool is_on_approach(void) const;
    void Log(void) const;

    //private helpers
    float predict_travel_distance(const Vector3f wind, const float height, const bool print);

    #define VTOL_LOITER_ALT_TOLERANCE 10.0f
};
