/*
  additional arming checks for plane
 */
#include "AP_Arming.h"
#include "Plane.h"

const AP_Param::GroupInfo AP_Arming_Plane::var_info[] = {
    // variables from parent vehicle
    AP_NESTEDGROUPINFO(AP_Arming, 0),

    // index 3 was RUDDER and should not be used

    AP_GROUPEND
};

/*
  additional arming checks for plane

 */
bool AP_Arming_Plane::pre_arm_checks(bool display_failure)
{
    // call parent class checks
    bool ret = AP_Arming::pre_arm_checks(display_failure);

    // Check airspeed sensor
    ret &= AP_Arming::airspeed_checks(display_failure);

    if (plane.g.fs_timeout_long < plane.g.fs_timeout_short && plane.g.fs_action_short != FS_ACTION_SHORT_DISABLED) {
        check_failed(ARMING_CHECK_NONE, display_failure, "FS_LONG_TIMEOUT < FS_SHORT_TIMEOUT");
        ret = false;
    }

    if (plane.aparm.roll_limit_cd < 300) {
        check_failed(ARMING_CHECK_NONE, display_failure, "LIM_ROLL_CD too small (%u)", plane.aparm.roll_limit_cd);
        ret = false;
    }

    if (plane.aparm.pitch_limit_max_cd < 300) {
        check_failed(ARMING_CHECK_NONE, display_failure, "LIM_PITCH_MAX too small (%u)", plane.aparm.pitch_limit_max_cd);
        ret = false;
    }

    if (plane.aparm.pitch_limit_min_cd > -300) {
        check_failed(ARMING_CHECK_NONE, display_failure, "LIM_PITCH_MIN too large (%u)", plane.aparm.pitch_limit_min_cd);
        ret = false;
    }

    if (plane.channel_throttle->get_reverse() && 
        plane.g.throttle_fs_enabled &&
        plane.g.throttle_fs_value < 
        plane.channel_throttle->get_radio_max()) {
        check_failed(ARMING_CHECK_NONE, display_failure, "Invalid THR_FS_VALUE for rev throttle");
        ret = false;
    }

    if (plane.quadplane.available() && plane.scheduler.get_loop_rate_hz() < 100) {
        check_failed(ARMING_CHECK_NONE, display_failure, "quadplane needs SCHED_LOOP_RATE >= 100");
        ret = false;
    }

    if (plane.quadplane.corvo_takeoff_inhibit()) {
        check_failed(ARMING_CHECK_NONE, display_failure, "Incorrect mode for takeoff");
        ret = false;
    }

    if (!plane.create_default_mission()) {
        check_failed(ARMING_CHECK_NONE, display_failure, "Mission Creation Failed");
        ret = false;
    }

    if (plane.control_mode == AUTO && plane.mission.num_commands() <= 1) {
        check_failed(ARMING_CHECK_NONE, display_failure, "No mission loaded");
        ret = false;
    }

    // check adsb avoidance failsafe
    if (plane.failsafe.adsb) {
        check_failed(ARMING_CHECK_NONE, display_failure, "ADSB threat detected");
        ret = false;
    }

#if HAVE_PX4_MIXER
    if (plane.last_mixer_crc == -1) {
        check_failed(ARMING_CHECK_NONE, display_failure, "Mixer error");
        ret = false;
    }
#endif // CONFIG_HAL_BOARD

    return ret;
}

bool AP_Arming_Plane::ins_checks(bool display_failure)
{
    // call parent class checks
    if (!AP_Arming::ins_checks(display_failure)) {
        return false;
    }

    // additional plane specific checks
    if ((checks_to_perform & ARMING_CHECK_ALL) ||
        (checks_to_perform & ARMING_CHECK_INS)) {
        if (!AP::ahrs().healthy()) {
            const char *reason = AP::ahrs().prearm_failure_reason();
            if (reason == nullptr) {
                reason = "AHRS not healthy";
            }
            check_failed(ARMING_CHECK_INS, display_failure, "%s", reason);
            return false;
        }
    }

    return true;
}

void AP_Arming_Plane::check_shake_to_arm(void)
{
        /*
     Special case handling of Corvo X - shake up and down TKOFF_ACCEL_CNT times to place into AUTO mode and arm
    */
    if (plane.quadplane.is_corvo_x_control_type() && RC_Channels::has_active_overrides() && !plane.arming.is_armed()) {
        /*
        This section of code enables the vehicle to be placed into AUTO, armed and flow with a default mission plan without
        requiring any operator interaction with the hand controller or GCS. To do this the operator needs to follow a sequence:
        1) Wait for arming checks to pass which will be indicated by the servos becoming active and the motors pointing up.
        2) Hold the vehicle nose up and rotate approximately +-45 degrees around the X axis within a 3 second period. This
           helps to prevent false triggering of the shake to launch function and also improves likelihood that a bad magnetomer
           will be detected prior to arming.
        3) Within 5 seconds of starting 2), shake the vehicle up and down TKOFF_ACCEL_CNT times at faster than 2 shakes per second
        The vehicle will then immediately enter AUTO with atime delay set by TKOFF_THR_DELAY before arming and starting motors. A
        mission plan will be automatically generated if not already loaded. If motors cannot be armed then the vehicle will remain
        in AUTO.
        */

        // This function requires that the vehicle be in a VTOL mode
        // If not then place in QLOITER
        if (!(plane.quadplane.in_vtol_auto() || plane.quadplane.in_vtol_mode())) {
            plane.set_mode(QLOITER, MODE_REASON_SHAKE_TO_LAUNCH);
        }

        uint32_t now_ms = millis();

        // calculate a delta time
        float dt = 0.001f * (float)(now_ms - shake_to_fly.last_time_ms);
        shake_to_fly.last_time_ms = now_ms;
        dt = constrain_float(dt, 0.0f, 0.02f);

        // integrate the X gyro and washout resulting angle to that it decays to zero over a 1 second time constant
        Vector3f gyro_rate = plane.ahrs.get_gyro();
        shake_to_fly.ang_rot_x_hpf += gyro_rate.x * dt;
        const float ang_lim_rad = radians(45.0f);
        shake_to_fly.ang_rot_x_hpf = constrain_float(shake_to_fly.ang_rot_x_hpf, -ang_lim_rad, ang_lim_rad);
        const float hpf_tconst = 1.0f;
        float hpf_coef = 1.0f - (dt / hpf_tconst);
        shake_to_fly.ang_rot_x_hpf *= hpf_coef;

        // Record RH and LH rotaton events
        if (shake_to_fly.ang_rot_x_hpf > (0.7f * ang_lim_rad)) {
            shake_to_fly.ang_x_rh_time_ms = now_ms;
        } else if (shake_to_fly.ang_rot_x_hpf < -(0.7f * ang_lim_rad)) {
            shake_to_fly.ang_x_lh_time_ms = now_ms;
        }

        // require each consecutive up/down shake event to be no more than this time apart
        const uint32_t shake_interval_ms = 500;

        // reset the detector if the rotors are not pointing up or the wing chord line is too far from vertical
        bool oriented_for_flight = (fabsf(plane.quadplane.ahrs_view->roll) <= degrees(15.0f)) // vehicle is not excessively rolled
                && (fabsf(plane.quadplane.ahrs_view->get_pitch_312_rotor()) <= degrees(10.0f)) // rotors are not excessively pitched
                && fabsf(plane.quadplane.ahrs_view->get_pitch_312_wing()) < radians (10.0f); // wing chord line is close to vertical
        if ((shake_to_fly.first_shake_time_ms != 0) && !oriented_for_flight) {
            shake_to_fly = {};
        }

        // calculate the vertical g after removing gravity and applying some noise filtering (positive is up)
        // set up the LPF so that there are 10 time constants for each shake up/down event
        Vector3f accel_ef = plane.ahrs.get_accel_ef_blended();
        float lpf_tconst = (0.1f * 0.001f) * (float)shake_interval_ms;
        lpf_tconst = MAX(lpf_tconst, dt);
        float lpf_coef = dt / lpf_tconst;
        shake_to_fly.accel_up_filt = (1.0f - lpf_coef) * shake_to_fly.accel_up_filt - lpf_coef * (GRAVITY_MSS + accel_ef.z);

        // detect up movement - require yaw rotation sequence to be completed recently for first shake to register
        bool yaw_rotation_is_recent = ((millis() - shake_to_fly.ang_x_rh_time_ms) < 5000) && ((millis() - shake_to_fly.ang_x_lh_time_ms) < 5000);
        if ((shake_to_fly.accel_up_filt > plane.g.takeoff_throttle_min_accel) &&
                (((shake_to_fly.up_shake_count == 0) && yaw_rotation_is_recent) || ((now_ms - shake_to_fly.up_shake_time_ms) < shake_interval_ms))) {
            if (shake_to_fly.up_shake_count == 0 && shake_to_fly.down_shake_count == 0) {
                // record start of shake sequence
                shake_to_fly.first_shake_time_ms = now_ms;
            }
            if (shake_to_fly.up_shake_count <= shake_to_fly.down_shake_count) {
                shake_to_fly.up_shake_count++;
                shake_to_fly.up_shake_time_ms = now_ms;
            }
         }

        // detect down movement - require yaw rotation sequence to be completed recently for first shake to register
        if ((shake_to_fly.accel_up_filt < -plane.g.takeoff_throttle_min_accel) &&
                (((shake_to_fly.down_shake_count == 0) && yaw_rotation_is_recent) || ((now_ms - shake_to_fly.down_shake_time_ms) < shake_interval_ms))) {
            if ((shake_to_fly.up_shake_count == 0) && (shake_to_fly.down_shake_count == 0)) {
                // start of shake sequence
                shake_to_fly.first_shake_time_ms = now_ms;
            }
            if (shake_to_fly.down_shake_count <= shake_to_fly.up_shake_count) {
                shake_to_fly.down_shake_count++;
                shake_to_fly.down_shake_time_ms = now_ms;
            }
        }

        uint32_t max_check_duration_ms = (uint32_t)plane.g2.takeoff_throttle_accel_count * shake_interval_ms;
        if (((now_ms - shake_to_fly.first_shake_time_ms) > max_check_duration_ms) // too much time since first shake
                && (shake_to_fly.first_shake_time_ms != 0) // not waiting for first shake
                && (shake_to_fly.shake_pass_time_ms == 0)) { // not delaying between selecting AUTO and arming
            // reset counters if motion not completed within required time
            shake_to_fly = {};
        } else if ((shake_to_fly.shake_pass_time_ms == 0)
                   && (shake_to_fly.up_shake_count >= plane.g2.takeoff_throttle_accel_count)
                   && (shake_to_fly.down_shake_count >= plane.g2.takeoff_throttle_accel_count)) {
            // if completed enough shakes, then record test completion time and place vehicle into AUTO mode
            shake_to_fly.shake_pass_time_ms = now_ms;
            plane.set_mode(AUTO, MODE_REASON_SHAKE_TO_LAUNCH);
        }

        // wait before arming - gives operator time to adjust grip and level vehicle before motors start
        // also allows time for flight mode change initialisation functions to complete
        // controlled by TKOFF_THR_DELAY parameter
        if ((shake_to_fly.shake_pass_time_ms != 0)
                && (plane.control_mode == AUTO) && ((now_ms - shake_to_fly.shake_pass_time_ms) > 100 * (uint32_t)plane.g.takeoff_throttle_delay)
                && oriented_for_flight) {
            // final sanity check that we will do a VTOL takeoff when armed
            bool has_valid_mission = false;
            AP_Mission::Mission_Command cmd = {};
            if (plane.mission.get_next_nav_cmd(1, cmd)) {
                has_valid_mission = (cmd.id == MAV_CMD_NAV_VTOL_TAKEOFF);
            }
            if (has_valid_mission) {
                if(plane.arm_motors(AP_Arming::ArmingMethod::SHAKE, true)) {
                    shake_arm_time_ms = now_ms;
                }
            }
            shake_to_fly = {};        }
    } else if (shake_to_fly.shake_pass_time_ms != 0) {
        // ensure that all test variables are reset when not in use
        shake_to_fly = {};
    }

    // The operator has within 5 seconds of arming to cancel by tilting in roll past 60 degrees
    if (plane.arming.is_armed() && plane.quadplane.in_vtol_auto() && ((millis() - shake_arm_time_ms) < 5000)) {
        const Matrix3f &rotMat = plane.ahrs.get_rotation_body_to_ned();
        // Check the magnitude of the DCM matrix that rotates the Y body axis component into the Z earth frame.
        // If this is large then the vehicle is rolled
        const float sin_60 = sinf(radians(60.0f));
        if (fabsf(rotMat.c.y) > sin_60) {
            plane.disarm_motors();
            shake_arm_time_ms = 0;
        }
    }
}