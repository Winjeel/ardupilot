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

/*
 *   AP_Landing_VTOL.cpp - Landing logic handler for ArduPlane for wind adaptive VTOL landing entry
 */

#include "AP_Landing.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>

// table of user settable parameters for wind adaptive VTOL landing entry
const AP_Param::GroupInfo AP_Landing_VTOL::var_info[] = {

    // @Param: DECEL
    // @DisplayName: Rate of deceleration
    // @Description: Deceleration rate assumed during transition to VTOL mode.
    // @Range: 0.0 10.0
    // @Units m/s/s
    // @User: Advanced
    AP_GROUPINFO("DECEL", 1, AP_Landing_VTOL, _vtol_decel, 3.0f),

    // @Param: XWIND
    // @DisplayName: Maximum allowed cross wind during VTOL transition
    // @Description: If the cross wind conponent is greater than this, a circular approach with into wind exit will be used.
    // @Range: 0.0 15.0
    // @Units m/s
    // @User: Advanced
    AP_GROUPINFO("XWIND", 2, AP_Landing_VTOL, _vtol_xwind_max, 4.0f),

    // @Param: TWIND
    // @DisplayName: Maximum allowed tail wind during VTOL transition
    // @Description: If the tail wind conponent is greater than this, a circular approach with into wind exit will be used.
    // @Range: 0.0 15.0
    // @Units m/s
    // @User: Advanced
    AP_GROUPINFO("TWIND", 3, AP_Landing_VTOL, _vtol_twind_max, 3.0f),

    AP_GROUPEND
};


// if DEBUG_PRINTS is defined statustexts will be sent to the GCS for debug purposes
// #define DEBUG_PRINTS

void AP_Landing_VTOL::do_land(const AP_Mission::Mission_Command& cmd, const float relative_altitude)
{
    _stage = VTOL_STAGE_FLY_TO_LANDING;

    // load the landing point in, the rest of path building is deferred for a better wind estimate
    memcpy(&_landing_point, &cmd.content.location, sizeof(Location));

    if (!_landing_point.flags.relative_alt && !_landing_point.flags.terrain_alt) {
        _approach_alt_offset = cmd.p1;
        _landing_point.alt += _approach_alt_offset * 100;
    } else {
        _approach_alt_offset = 0.0f;
    }

    // specify the loiter centre which is offset at 90 degrees to the wind by the loiter radius and
    // in the direction of wind by the decel distance.

    //Set the loiter centre to the landing waypoint initially before applying the offsets
    memcpy(&_loiter_point, &_landing_point, sizeof(Location));

    // Use the wind and nominal flight speed to estimate the into wind approach ground speed
    Vector3f wind = landing.ahrs.wind_estimate();
    float wind_speed = sqrtf(wind.x*wind.x+wind.y*wind.y);
    float true_airspeed = 0.01f*landing.aparm.airspeed_cruise_cm * landing.ahrs.get_EAS2TAS();
    float approach_gndspeed = MAX(true_airspeed - wind_speed, 0.0f);

    // Calculate position offset from the landing point to start of braking/transition maneouvre
    _predicted_decel_distance = 0.5f * approach_gndspeed * approach_gndspeed / _vtol_decel;
    float vec_arg = atan2f(-wind.y, -wind.x);
    Vector2f offset_vec = {-_predicted_decel_distance * cosf(vec_arg) , -_predicted_decel_distance * sinf(vec_arg)};
    vec_arg += _loiter_point.flags.loiter_ccw ? radians(90.0f) : radians(-90.0f);

    // Increase the loiter radius if necessary to prevent bank angle saturating
    _loiter_radius = landing.nav_controller->loiter_radius(landing.aparm.loiter_radius);

    // calculate the radial offset and add to positon offset from landing point to loiter centre
    offset_vec.x -= _loiter_radius * cosf(vec_arg);
    offset_vec.y -= _loiter_radius * sinf(vec_arg);

    // Apply total offset to get loiter centre
    location_offset(_loiter_point, offset_vec.x, offset_vec.y);

    // initialise current location
    Location current_loc;
    landing.ahrs.get_position(current_loc);

    // initialise control variables
    _low_wind_overshoot = false;
    _use_loiter = false;
    _use_direct = false;
    _timeout_count = 0;

    // Iniitalise VTOL entry variables
    _target_heading_deg = degrees(vec_arg);
    _entry_point = current_loc;
    _approach_vec_unit = location_diff(_entry_point,_landing_point);
    _approach_vec_unit.normalize();
}

// currently identical to the slope aborts
void AP_Landing_VTOL::verify_abort_landing(const Location &prev_WP_loc, Location &next_WP_loc, bool &throttle_suppressed)
{
    // when aborting a landing, mimic the verify_takeoff with steering hold. Once
    // the altitude has been reached, restart the landing sequence
    throttle_suppressed = false;
    landing.nav_controller->update_heading_hold(get_bearing_cd(prev_WP_loc, next_WP_loc));
}


/*
  update navigation for landing
 */
bool AP_Landing_VTOL::verify_land(const Location &prev_WP_loc, Location &next_WP_loc, const Location &current_loc,
        const float height, const float sink_rate, const float wp_proportion, const uint32_t last_flying_ms,
        const bool is_armed, const bool is_flying, const bool rangefinder_state_in_range)
{
    switch (_stage) {
    case VTOL_STAGE_FLY_TO_LANDING:
        {
            // get component of wind velocity perpendicular to and along track to landing point
            Vector2f vec_to_land_unit = location_diff(current_loc,_landing_point);
            vec_to_land_unit.normalize();
            Vector3f wind = landing.ahrs.wind_estimate();
            float tail_wind = vec_to_land_unit.x * wind.x + vec_to_land_unit.y * wind.y;
            float cross_wind = fabsf(wind.x * vec_to_land_unit.y - wind.y * vec_to_land_unit.x);
            if (!_use_direct &&
                    (tail_wind > _vtol_twind_max ||
                    cross_wind > _vtol_xwind_max ||
                    _low_wind_overshoot ||
                    _use_loiter)) {
                _use_loiter = true;
                // Do an offset loiter recovery so that the transition to VTOL
                // is into wind and close to the landing waypoint
                next_WP_loc.lat = _loiter_point.lat;
                next_WP_loc.lng = _loiter_point.lng;
                if (get_distance(current_loc, _loiter_point) > fabsf(2 * _loiter_radius)) {
                    landing.nav_controller->update_waypoint(current_loc, _loiter_point);
                    _timeout_count = 0;
                    return false;
                }
                gcs().send_text(MAV_SEVERITY_INFO, "Recovery - Joining %4.1f m Loiter",_loiter_radius);
                gcs().send_text(MAV_SEVERITY_INFO, "Tail,Cross Wind = %4.1f , %4.1f m/s",tail_wind,cross_wind);
                _stage = VTOL_STAGE_HEIGHT_CAPTURE;
                _loiter_sum_cd = 0; // reset the loiter counter
            } else {
                // fly straight to the waypoint and transition

                // abort into a loiter recovery if a go around is commanded
                if (_commanded_go_around) {
                    gcs().send_text(MAV_SEVERITY_INFO, "Recovery - Go Around Set");
                    _low_wind_overshoot = true;
                    next_WP_loc.lat = _loiter_point.lat;
                    next_WP_loc.lng = _loiter_point.lng;
                    return false;
                }

                // calculate the along track speed to the landing point
                Vector2f velNE = landing.ahrs.groundspeed_vector();
                float closing_speed_to_land = _approach_vec_unit.x * velNE.x + _approach_vec_unit.y * velNE.y;

                // calculate the estimated distance reauired to stop during VTOL transition
                _predicted_decel_distance = 0.5f * closing_speed_to_land * closing_speed_to_land / _vtol_decel;

                // calculate the location where we need to commence the VTOL transition allowing for braking distance
                Location transition_point = _landing_point;
                location_offset(transition_point, -_predicted_decel_distance * _approach_vec_unit.x, -_predicted_decel_distance * _approach_vec_unit.y);

                // calculate closing speed to transition point
                Vector2f delta_to_transition = location_diff(current_loc,transition_point);
                delta_to_transition.normalize();
                float closing_speed_to_transition = delta_to_transition.x * velNE.x + delta_to_transition.y * velNE.y;

                // create an extended waypoint 1000m beyond the landing point so we always have something
                // to navigate to
                _extended_approach = _landing_point;
                location_offset(_extended_approach, 1000.0f * _approach_vec_unit.x, 1000.0f * _approach_vec_unit.y);

                // continue to capture and follow the track to the waypoint extension
                next_WP_loc.lat = _extended_approach.lat;
                next_WP_loc.lng = _extended_approach.lng;
                landing.nav_controller->update_waypoint(current_loc, _extended_approach);
                _timeout_count = 0;

                // calculate the closing speed to the extended waypoint
                Vector2f delta_to_ext = location_diff(current_loc,_extended_approach);
                delta_to_ext.normalize();
                float closing_speed_to_ext = delta_to_ext.x * velNE.x + delta_to_ext.y * velNE.y;

                if (closing_speed_to_transition < 0.0f && closing_speed_to_ext > 0.0f) {
                    // passed transition waypoint - check if missed
                    float distance_to_transition = delta_to_transition.length();
                    bool not_at_height = ((height - _approach_alt_offset) > VTOL_LOITER_ALT_TOLERANCE);
                    bool missed_waypoint = (distance_to_transition > fabsf(_loiter_radius)) &&
                                            (closing_speed_to_ext > 0.0f);
                    if (not_at_height || missed_waypoint) {
                        // we have missed the waypoint so now must use loiter method
                        _low_wind_overshoot = true;
                        next_WP_loc.lat = _loiter_point.lat;
                        next_WP_loc.lng = _loiter_point.lng;
                        gcs().send_text(MAV_SEVERITY_INFO, "Recovery - Overshot WP - Using Loiter");
                        return false;
                    }
                } else {
                    // continue to capture and follow straight line that passes over the landing point
                    return false;
                }

                // proceed directly to a VTOL transition
                _use_direct = true;
                next_WP_loc.lat = _landing_point.lat;
                next_WP_loc.lng = _landing_point.lng;
                _stage = VTOL_STAGE_LAND_VTOL;
                gcs().send_text(MAV_SEVERITY_INFO, "Recovery - Low Wind Direct");
                gcs().send_text(MAV_SEVERITY_INFO, "Tail,Cross Wind = %4.1f , %4.1f m/s",tail_wind,cross_wind);
                return true;
            }
        }
        FALLTHROUGH;
    case VTOL_STAGE_HEIGHT_CAPTURE:
        {
            landing.nav_controller->update_loiter(_loiter_point, _loiter_radius, _loiter_point.flags.loiter_ccw ? -1 : 1, Vector2f(0.0f,0.0f));
            _timeout_count = 0;
            if (!landing.nav_controller->reached_loiter_target() || ((height - _approach_alt_offset) > VTOL_LOITER_ALT_TOLERANCE)) {
                // wait until the altitude is correct before considering a breakout
                return false;
            }
            _last_target_bearing = landing.nav_controller->target_bearing_cd();
            _stage = VTOL_STAGE_WAIT_FOR_BREAKOUT;
            gcs().send_text(MAV_SEVERITY_INFO, "Recovery - Adjusting Loiter");
        }
        FALLTHROUGH;
    case VTOL_STAGE_WAIT_FOR_BREAKOUT:
        {
            landing.nav_controller->update_loiter(_loiter_point, _loiter_radius, _loiter_point.flags.loiter_ccw ? -1 : 1, Vector2f(0.0f,0.0f));
            _timeout_count = 0;
            // Ensure we have done a full circle before landing to give a reliable approach angle.
            if ((_loiter_sum_cd < 36000) && !_low_wind_overshoot) {
                // Adjust the loiter centre
                memcpy(&_loiter_point, &_landing_point, sizeof(Location));
                Vector3f wind = landing.ahrs.wind_estimate();
                float approach_speed = MAX(15.0f - sqrtf(wind.x*wind.x+wind.y*wind.y), 0.0f);
                float decel_distance = 0.5f * approach_speed * approach_speed / _vtol_decel;
                float vec_arg = atan2f(-wind.y, -wind.x);
                _target_heading_deg = degrees(vec_arg);
                Vector2f offset_vec = {-decel_distance * cosf(vec_arg) , -decel_distance * sinf(vec_arg)};
                vec_arg += _loiter_point.flags.loiter_ccw ? radians(90.0f) : radians(-90.0f);
                offset_vec.x -= _loiter_radius * cosf(vec_arg);
                offset_vec.y -= _loiter_radius * sinf(vec_arg);
                location_offset(_loiter_point, offset_vec.x, offset_vec.y);
                next_WP_loc.lat = _loiter_point.lat;
                next_WP_loc.lng = _loiter_point.lng;
                int32_t target_bearing = landing.nav_controller->target_bearing_cd();
                int32_t delta = wrap_180_cd(target_bearing - _last_target_bearing);
                _last_target_bearing = target_bearing;
                if (delta > 0) {
                    _loiter_sum_cd += delta;
                } else {
                    _loiter_sum_cd -= delta;
                }
                return false;
            }

            if (_commanded_go_around) {
                if (_loiter_sum_cd < 36000) {
                    // If a go around has been commanded do a minimum of one complete turn
                    return false;
                } else {
                    // clear the go around
                    _commanded_go_around = false;
                    gcs().send_text(MAV_SEVERITY_INFO, "Recovery - Go Around Cleared");
                }
            }

            Vector2f velNE = landing.ahrs.groundspeed_vector();
            float course_deg = degrees(atan2f(velNE.y, velNE.x));
            if (fabsf(wrap_180(course_deg - _target_heading_deg)) > 1.0f) {
                return false;
            }
            gcs().send_text(MAV_SEVERITY_INFO, "Recovery - Exiting Loiter at %4.1f deg",_target_heading_deg);
            _stage = VTOL_STAGE_LAND_VTOL;
            next_WP_loc.lat = _landing_point.lat;
            next_WP_loc.lng = _landing_point.lng;
            return true;
        }
        FALLTHROUGH;
    case VTOL_STAGE_LAND_VTOL:
        {
            // Set next waypoint to landing point.
            next_WP_loc.lat = _landing_point.lat;
            next_WP_loc.lng = _landing_point.lng;

            // Command loiter in case switch out of landing times out - this should never happen
            _timeout_count += 1;
            if (_timeout_count > 10) {

                landing.nav_controller->update_loiter(_landing_point, _loiter_radius, _loiter_point.flags.loiter_ccw ? -1 : 1, Vector2f(0.0f,0.0f));
            }

            return true;
        }
    default:
        {
            // It should never get here, but just in case put the vehicle back into loiter
            next_WP_loc.lat = _loiter_point.lat;
            next_WP_loc.lng = _loiter_point.lng;
            _timeout_count += 1;
            if (_timeout_count > 10) {
                _stage = VTOL_STAGE_WAIT_FOR_BREAKOUT;
                _low_wind_overshoot = true;
                _loiter_sum_cd = 0;
                landing.nav_controller->update_loiter(_loiter_point, _loiter_radius, _loiter_point.flags.loiter_ccw ? -1 : 1, Vector2f(0.0f,0.0f));
            }

            return false;
        }
    }
}

bool AP_Landing_VTOL::request_go_around(void)
{
    _commanded_go_around =true;
    _loiter_sum_cd = 0;
    return true;
}

bool AP_Landing_VTOL::is_flying_forward(void) const
{
    return _stage != VTOL_STAGE_LAND_VTOL;
}

bool AP_Landing_VTOL::is_on_approach(void) const
{
    return _stage == VTOL_STAGE_LAND_VTOL;
}

bool AP_Landing_VTOL::get_target_altitude_location(Location &location)
{
    memcpy(&location, &_landing_point, sizeof(Location));
    return true;
}

int32_t AP_Landing_VTOL::get_target_airspeed_cm(void) const
{
    if (_stage == VTOL_STAGE_LAND_VTOL) {
        return landing.pre_flare_airspeed * 100;
    } else {
        return landing.aparm.airspeed_cruise_cm;
    }
}

void AP_Landing_VTOL::Log(void) const {
    struct log_DSTL pkt = {
        LOG_PACKET_HEADER_INIT(LOG_DSTL_MSG),
        time_us          : AP_HAL::micros64(),
        stage            : (uint8_t)_stage,
        target_heading   : _target_heading_deg,
        target_lat       : _landing_point.lat,
        target_lng       : _landing_point.lng,
        target_alt       : _landing_point.alt,
        travel_distance  : (int16_t)constrain_float(_predicted_decel_distance * 1e2f, (float)INT16_MIN, (float)INT16_MAX),
    };
    DataFlash_Class::instance()->WriteBlock(&pkt, sizeof(pkt));
}

float AP_Landing_VTOL::predict_travel_distance(const Vector3f wind, const float height, const bool print)
{
    Vector2f velNE = landing.ahrs.groundspeed_vector();
    return 0.5f * (velNE.x * velNE.x + velNE.y * velNE.y) / _vtol_decel;
}
