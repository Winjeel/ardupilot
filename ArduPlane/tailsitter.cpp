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
  control code for tailsitters. Enabled by setting Q_FRAME_CLASS=10 or 14 when motors can be vectored in pitch
 */

#include "Plane.h"

/*
  return true when flying a tailsitter or TVBS
 */
bool QuadPlane::is_tailsitter(void) const
{
    return available() && (frame_class == AP_Motors::MOTOR_FRAME_TAILSITTER || frame_class == AP_Motors::MOTOR_FRAME_TVBS);
}

/*
  check if we are flying as a tailsitter
 */
bool QuadPlane::tailsitter_active(void)
{
    if (!is_tailsitter()) {
        return false;
    }
    if (in_vtol_mode()) {
        return true;
    }
    // check if we are in ANGLE_WAIT fixed wing transition
    if (transition_state == TRANSITION_ANGLE_WAIT_FW) {
        return true;
    }
    return false;
}

/*
  run output for tailsitters
 */
void QuadPlane::tailsitter_output(void)
{
    if (!is_tailsitter()) {
        return;
    }
    if (!tailsitter_active() || reverse_transition_pullup_active) {
        if (tailsitter.vectored_forward_gain > 0) {
            // thrust vectoring in fixed wing flight
            float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
            float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
            float tilt_left  = (elevator + aileron) * tailsitter.vectored_forward_gain;
            float tilt_right = (elevator - aileron) * tailsitter.vectored_forward_gain;
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
        } else {
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, 0);
            SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, 0);
        }
        if (in_tailsitter_vtol_transition() && !throttle_wait && is_flying() && hal.util->get_soft_armed()) {
            /*
              during transitions to vtol mode set the throttle to the
              hover throttle, and set the altitude controller
              integrator to the same throttle level
             */
            uint8_t throttle = motors->get_throttle_hover() * 100;
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, throttle);
            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, 0);
            pos_control->get_accel_z_pid().set_integrator(throttle*10);
        }
        return;
    }
    
    motors_output(false);
    plane.pitchController.reset_I();
    plane.rollController.reset_I();

    if (hal.util->get_soft_armed()) {
        // scale surfaces for throttle
        tailsitter_speed_scaling();
    }

    
    if (tailsitter.vectored_hover_gain > 0) {
        // thrust vectoring VTOL modes
        float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
        float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);
        /*
          apply extra elevator when at high pitch errors, using a
          power law. This allows the motors to point straight up for
          takeoff without integrator windup
         */
        int32_t pitch_error_cd = (plane.nav_pitch_cd - ahrs_view->pitch_sensor) * 0.5;
        float extra_pitch = constrain_float(pitch_error_cd, -4500, 4500) / 4500.0;
        float extra_sign = extra_pitch > 0?1:-1;
        float extra_elevator = extra_sign * powf(fabsf(extra_pitch), tailsitter.vectored_hover_power) * 4500;
        float tilt_left  = extra_elevator + (elevator + aileron) * tailsitter.vectored_hover_gain;
        float tilt_right = extra_elevator + (elevator - aileron) * tailsitter.vectored_hover_gain;
        if (fabsf(tilt_left) >= 4500 || fabsf(tilt_right) >= 4500) {
            // prevent integrator windup
            motors->limit.roll_pitch = 1;
            motors->limit.yaw = 1;
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);
    }
    
    
    if (tailsitter.input_mask_chan > 0 &&
        tailsitter.input_mask > 0 &&
        RC_Channels::get_radio_in(tailsitter.input_mask_chan-1) > 1700) {
        // the user is learning to prop-hang
        if (tailsitter.input_mask & TAILSITTER_MASK_AILERON) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, plane.channel_roll->get_control_in_zero_dz());
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_ELEVATOR) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, plane.channel_pitch->get_control_in_zero_dz());
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_THROTTLE) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.get_throttle_input(true));
        }
        if (tailsitter.input_mask & TAILSITTER_MASK_RUDDER) {
            SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, plane.channel_rudder->get_control_in_zero_dz());
        }
    }
}


/*
  return true when we have completed enough of a transition to switch to fixed wing control
 */
bool QuadPlane::tailsitter_transition_fw_complete(void)
{
    if (plane.fly_inverted()) {
        // transition immediately
        return true;
    }

    int32_t roll_cd = labs(ahrs_view->roll_sensor);
    if (roll_cd > 9000) {
        roll_cd = 18000 - roll_cd;
    }

    // Calculate the wing pitch pitch angle below which we need to transition
    // This is required for TVBS airframes. Note the use of ahrs which returns a pitch
    // angle that is zero when the vehicle is fixed wing level flight
    uint16_t wing_pitch_transition_cd = 9000 - tailsitter.transition_angle*100;
    bool wing_criteria_met = ahrs.pitch_sensor < wing_pitch_transition_cd;

    // Check the amount of rotor tilt from vertical noting that for a conventional tailsitter
    // ahrs_view returns a pitch angle that is zero when the nose is pointing up, whereas for
    // a TVBS vehicle it returns the pitch tilt of the rotors and is zero when the rotors are
    // pointing up.
    bool rotor_criteria_met = labs(ahrs_view->pitch_sensor) > tailsitter.transition_angle*100;

    // Require both rotor and wing to have tilted. This is necessary because for a TVBS vehicle
    // the rotor can have tilted past the threshold before the wing is ready to fly.
    if ((wing_criteria_met && rotor_criteria_met) ||
        labs(ahrs_view->roll_sensor) > tailsitter.transition_angle*100 ||
        (AP_HAL::millis() - transition_start_ms) > uint32_t(transition_time_ms)) {
        return true;
    }

    // still waiting
    return false;
}


/*
  return true when we have completed the pullup in FW mode.
 */
bool QuadPlane::tailsitter_transition_pullup_complete(void) const
{
    if (transition_state != TRANSITION_ANGLE_WAIT_VTOL || !plane.is_flying()) {
        return true;
    }
    if (AP_HAL::millis() - transition_start_ms > 5000) {
        // limit time in FW decel pullup to 5 seconds
        return true;
    } else if (AP_HAL::millis() - transition_start_ms > 1000 &&
               labs(ahrs_view->roll_sensor) > 1500) {
        // After 1 second, start checking to see if we have lost roll control which would
        // indicate stall onset.
        return true;
    } else if (AP_HAL::millis() - transition_start_ms > 2000 &&
               (ahrs_view->pitch_sensor < 0 || inertial_nav.get_velocity_z() > 0.5f)) {
        // After 2 seconds which is long enough for the nose up pitch to take effect,
        // start checking to see if we are descending or pitched down which indicates stall onset.
        return true;
    } else {
        return false;
    }
}

/*
  return true when we have completed enough of a transition to switch to full VTOL control
 */
bool QuadPlane::tailsitter_transition_vtol_complete(void) const
{
    if (transition_state != TRANSITION_ANGLE_WAIT_VTOL || !plane.is_flying()) {
        return true;
    } else if (!tailsitter_transition_pullup_complete()) {
        return false;
    }
    Vector3f velNED;
    float fwd_spd = 0.0f;
    if (ahrs_view->get_velocity_NED(velNED)) {
        fwd_spd =  velNED.x * ahrs_view->cos_yaw() + velNED.y * ahrs_view->sin_yaw();
    }
    if (AP_HAL::millis() - transition_start_ms > (uint32_t)tailsitter.tvbs_bt_time_msec
            || fwd_spd < 1.0f
            || fabsf(ahrs_view->get_pitch_312_wing()) < radians (10.0f)) {
        return true;
    } else {
        return false;
    }
}

// handle different tailsitter input types
void QuadPlane::tailsitter_check_input(void)
{
    if (tailsitter_active() &&
        tailsitter.input_type == TAILSITTER_INPUT_PLANE) {
        // the user has asked for body frame controls when tailsitter
        // is active. We switch around the control_in value for the
        // channels to do this, as that ensures the value is
        // consistent throughout the code
        int16_t roll_in = plane.channel_roll->get_control_in();
        int16_t yaw_in = plane.channel_rudder->get_control_in();
        plane.channel_roll->set_control_in(yaw_in);
        plane.channel_rudder->set_control_in(-roll_in);
    }
}

/*
  return true if we are a tailsitter transitioning to VTOL flight  
 */
bool QuadPlane::in_tailsitter_vtol_transition(void) const
{
    return is_tailsitter() &&
            in_vtol_mode() &&
            transition_state == TRANSITION_ANGLE_WAIT_VTOL &&
            plane.is_flying();
}

/*
  account for speed scaling of control surfaces in hover
*/
void QuadPlane::tailsitter_speed_scaling(void)
{
    if (frame_class == AP_Motors::MOTOR_FRAME_TVBS) {
        return;
    }
    const float hover_throttle = motors->get_throttle_hover();
    const float throttle = motors->get_throttle();
    float scaling;

    if (is_zero(throttle)) {
        scaling = tailsitter.throttle_scale_max;
    } else {
        scaling = constrain_float(hover_throttle / throttle, 0, tailsitter.throttle_scale_max);
    }

    const SRV_Channel::Aux_servo_function_t functions[2] = {
        SRV_Channel::Aux_servo_function_t::k_aileron,
        SRV_Channel::Aux_servo_function_t::k_elevator};
    for (uint8_t i=0; i<ARRAY_SIZE(functions); i++) {
        int32_t v = SRV_Channels::get_output_scaled(functions[i]);
        v *= scaling;
        v = constrain_int32(v, -SERVO_MAX, SERVO_MAX);
        SRV_Channels::set_output_scaled(functions[i], v);
    }
}
    
/*
  run output for twin vectored belly sitters
*/
void QuadPlane::tvbs_output(void)
{
    if (!is_tailsitter()) {
        tvbs_active = false;
        return;
    }

    // Define body relative thrust angle limits in degrees.
    float lower_ang_limit = (float)tailsitter.tvbs_ang_min_deg;
    float upper_ang_limit = (float)tailsitter.tvbs_ang_max_deg;

    // get control deflections in equivalent centi-degrees
    float aileron = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron);
    float elevator = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator);

    if (!tailsitter_active() || reverse_transition_pullup_active) {
        // calculate aileron thrust vectoring deflection
        float ail_dz_cd = 100.0f * tailsitter.tvbs_fw_ail_deadband_deg; // aileron thrust vectoring deadzone (centi-deg)
        float ail_thrustvec_cd; // thrust vectoring aileron (centi-deg)
        if (aileron > ail_dz_cd) {
            ail_thrustvec_cd = tailsitter.tvbs_fw_ail_fwd_gain * (aileron - ail_dz_cd);
        } else if (aileron < -ail_dz_cd) {
            ail_thrustvec_cd = tailsitter.tvbs_fw_ail_fwd_gain * (aileron + ail_dz_cd);
        } else {
            ail_thrustvec_cd = 0.0f;
        }

        // Calculate elevator thrust vectoring deflection
        float elev_dz_cd = 100.0f * tailsitter.tvbs_fw_elev_deadband_deg; // elevator thrust vectoring deadzone (centi-deg)
        float elev_thrustvec_cd; // thrust vectoring elevator (centi-deg)
        if (elevator > elev_dz_cd) {
            elev_thrustvec_cd = tailsitter.tvbs_fw_elev_fwd_gain * (elevator - elev_dz_cd);
        } else if (elevator < -elev_dz_cd) {
            elev_thrustvec_cd = tailsitter.tvbs_fw_elev_fwd_gain * (elevator + elev_dz_cd);
        } else {
            elev_thrustvec_cd = 0.0f;
        }

        // Thrust vectoring in fixed wing flight
        float tilt_left  = elev_thrustvec_cd + ail_thrustvec_cd;
        float tilt_right = elev_thrustvec_cd - ail_thrustvec_cd;
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, tilt_left);
        SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, tilt_right);

        tvbs_active = false;

        // Get the current tilt servo demands
        float servo_angle_left_init = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorLeft);
        float servo_angle_right_init = SRV_Channels::get_output_scaled(SRV_Channel::k_tiltMotorRight);

        // Convert to body relative thrust angles

        const float scale_factor = 1.0f / 4500.0f;
        float thrust_angle_left_init = 0.0f; // angle of the left thrust-line relative to body in degrees
        if (servo_angle_left_init > 0.0f) {
            // Handle servo movement between trim and max pwm
            thrust_angle_left_init = scale_factor * servo_angle_left_init * upper_ang_limit;
        } else {
            // Handle servo movement between trim and min pwm
            thrust_angle_left_init = - scale_factor * servo_angle_left_init * lower_ang_limit;
        }

        float thrust_angle_right_init = 0.0f; // angle of the right thrust-line relative to body in degrees
        if (thrust_angle_right_init > 0.0f) {
            // Handle servo movement between trim and max pwm
            thrust_angle_right_init = scale_factor * servo_angle_right_init * upper_ang_limit;
        } else {
            // Handle servo movement between trim and min pwm
            thrust_angle_right_init = - scale_factor * servo_angle_right_init * lower_ang_limit;
        }

        // Estimate net rotor to body tilt angle in degrees
        tvbs_body_thrust_angle_dem = tvbs_body_thrust_angle_est = tvbs_body_thrust_angle_est_prev = 0.5f * (thrust_angle_left_init + thrust_angle_right_init);

        // Adjust the VTOL attitude view for the body relative rotor tilt
        ahrs_view->set_pitch_offset(-tvbs_body_thrust_angle_est);

        return;
    }

    // Check if we are transitioning into TVBS operation and initialise state variables
    // to prevent large tranients on entry that could destabilise the pitch control loop
    if (!tvbs_active) {
        // Reset the rotor pitch demand filter state variables
        tvbs_pitch_dem_filt_cd = tvbs_pitch_dem_cd = ahrs_view->pitch_sensor;
        tvbs_last_filt_time_ms = AP_HAL::millis();
        tvbs_pitch_rate_filt = degrees(ahrs_view->get_gyro().y);
        peak_tilt_rate_pos = 0.0f;
        peak_tilt_rate_neg = 0.0f;
        tvbs_thrust_ang_deriv = 0.0f;
        prev_pitch_error_deg = 0.0f;

        // Reset elevator channel gain limit cycle control variables
        _last_elev_feedback = 0.0f;
        _elev_slew_rate_filter.reset(0.0f);
        _elev_slew_rate_amplitude = 0.0f;
        _limit_cycle_gain_modifier = 1.0f;

        // record the transtion time from a non VTOL mode
        reverse_transition_time_ms = AP_HAL::millis();

    }

    // Keep plane control loop integrators zeroed
    plane.pitchController.reset_I();
    plane.rollController.reset_I();

    /*
     * Calculate a body frame thrust angle demand for each motor that keeps rotors at the demanded
     * pitch angle plus differential movement for equivalent aileron control.
    */

    // update the average time step used by all filters
    float delta_time = 0.001f * (float)(AP_HAL::millis() - tvbs_last_filt_time_ms);
    tvbs_last_filt_time_ms = AP_HAL::millis();
    tvbs_dt_avg = constrain_float(0.01f * delta_time + 0.99f * tvbs_dt_avg, 0.002f, 0.025f);

    // check for loss of attitude control and trigger a recovery mode that puts the nose in an up position with hover throttle
    // loss of control is characterised by large angular rates and a high descent rate
    float gyro_rate_length = ahrs_view->get_gyro().length();
    float alpha = 4.0f * tvbs_dt_avg;
    gyro_rate_length_filt = (1.0f - alpha) * gyro_rate_length_filt + alpha * gyro_rate_length;
    bool high_gyro_rate = gyro_rate_length_filt > radians(360.0f);
    Vector3f vel_NED;
    ahrs.get_velocity_NED(vel_NED);
    bool high_sink_rate = vel_NED.z > 5.0f;
    if (high_gyro_rate && high_sink_rate) {
        time_control_lost_ms = AP_HAL::millis();
    }
    if ((tailsitter.tvbs_ar_tune == 85) || (AP_HAL::millis() - time_control_lost_ms < 3000)) {
        if (!control_loss_declared) {
            hal.console->printf("TVBS attitude control lost");
            control_loss_declared = true;
        }
    } else {
        if (control_loss_declared) {
            hal.console->printf("TVBS attitude control regained");
            control_loss_declared = false;
            init_mode();
        }
    }

    if (control_loss_declared) {
        // Reset the rotor pitch demand filter state variables
        peak_tilt_rate_pos = 0.0f;
        peak_tilt_rate_neg = 0.0f;
        prev_pitch_error_deg = 0.0f;

        attitude_control->zero_roll_integrator();
        attitude_control->zero_pitch_integrator();
        attitude_control->zero_yaw_integrator();

        float recovery_throttle = constrain_float(motors->get_throttle_hover() + 0.1f, 0.5f, 0.9f);
        if (tailsitter.tvbs_ar_tune != 85) {
            attitude_control->set_throttle_out(recovery_throttle, true, 0);
        }

        lower_ang_limit = MAX(lower_ang_limit,-30.0f);
        upper_ang_limit = MIN(upper_ang_limit, 30.0f);
    }

    // Calculates the motor demands used to control thrust and Z-axis control moment
    // Also updates the equivalent rudder used by this control loop
    // Uses the ArduCopter attitude control loop outputs
    motors_output();

    // get rudder deflection in equivalent centi-degrees
    float rudder = SRV_Channels::get_output_scaled(SRV_Channel::k_rudder);

    // rate limit the demanded rotor pitch angle
    if (!control_loss_declared) {
        // keep rotors pointing up during VTOL control of back transition
        float rotor_pitch_demand_input;
        if (!reverse_transition_active) {
            rotor_pitch_demand_input = (float)plane.nav_pitch_cd;
        } else {
            rotor_pitch_demand_input = 0.0f;
            float recovery_throttle = constrain_float(motors->get_throttle_hover(), 0.5f, 0.9f);
            attitude_control->set_throttle_out(recovery_throttle, true, 0);
        }
        float delta_limit = 100.0f * (float)tailsitter.tvbs_slew_lim_dps * tvbs_dt_avg;
        if ((rotor_pitch_demand_input - tvbs_pitch_dem_cd) > delta_limit) {
            tvbs_pitch_dem_cd += delta_limit;
        } else if ((rotor_pitch_demand_input - tvbs_pitch_dem_cd) < -delta_limit) {
            tvbs_pitch_dem_cd -= delta_limit;
        } else {
            tvbs_pitch_dem_cd = rotor_pitch_demand_input;
        }
    }

    // apply a first order lowpass filter to the demand
    alpha = tvbs_dt_avg / MAX(0.001f * (float)tailsitter.tvbs_slew_tau_msec, tvbs_dt_avg);
    tvbs_pitch_dem_filt_cd = alpha * tvbs_pitch_dem_cd + (1.0f - alpha) * tvbs_pitch_dem_filt_cd;

    // calculate a correction term that compensates for servo lag and improves pitch damping
    float servo_lag_correction =  - 0.001f * (float)tailsitter.tvbs_tilt_lag_ms * degrees(ahrs_view->get_gyro().y);

    // Use the third rotation in a Tait-Bryan 312 sequence to measure pitch angle of wing
    float pitch_312_mea_rad = ahrs_view->get_pitch_312_wing();

    // Get demanded pitch of rotor
    float pitch_demand_rad = radians(0.01f * tvbs_pitch_dem_filt_cd);

    // calculate body frame thrust angle
    float pitch_error_deg = degrees(pitch_demand_rad - pitch_312_mea_rad);
    float pitch_delta_deg = pitch_error_deg - prev_pitch_error_deg;
    prev_pitch_error_deg = pitch_error_deg;

    // calculate peak positive and negtive slew rates using a peak hold filter with exponential decay
    alpha = 1.0f - (tvbs_dt_avg / MAX(tailsitter.elev_slew_rate_tau_sec, tvbs_dt_avg));
    if (pitch_delta_deg >= 0.0f) {
        peak_tilt_rate_pos = MAX(fabsf(pitch_delta_deg / tvbs_dt_avg),peak_tilt_rate_pos);
    } else {
        peak_tilt_rate_neg = MAX(fabsf(pitch_delta_deg / tvbs_dt_avg),peak_tilt_rate_neg);
    }
    peak_tilt_rate_pos *= alpha;
    peak_tilt_rate_neg *= alpha;

    // calculate a reduced gain if we have had exceedances in both directions or loss of control has been declared
    float tilt_gain_factor = 1.0f;
    bool tilt_saturated = false;
    if (control_loss_declared) {
        tilt_gain_factor = tailsitter.tvbs_ar_gain;
        tilt_saturated = true;
    } else if ((peak_tilt_rate_pos > tailsitter.tvbs_tilt_slew_lim_dps) && (peak_tilt_rate_neg > tailsitter.tvbs_tilt_slew_lim_dps)) {
        tilt_gain_factor = tailsitter.tvbs_tilt_slew_lim_dps / MIN(peak_tilt_rate_pos,peak_tilt_rate_neg);
        tilt_gain_factor = MAX(tilt_gain_factor, 0.1f);
        tilt_saturated = true;
    }

    // Correct the demanded body frame thrust angle and limit
    tvbs_body_thrust_angle_dem = servo_lag_correction + constrain_float(tilt_gain_factor * pitch_error_deg, lower_ang_limit, upper_ang_limit);
    tvbs_body_thrust_angle_dem = constrain_float(tvbs_body_thrust_angle_dem, lower_ang_limit, upper_ang_limit);

    // Estimate body frame thrust angle excluding effect of servo delay/rate term
    tvbs_body_thrust_angle_est = tilt_gain_factor * pitch_error_deg;
    tvbs_body_thrust_angle_est = constrain_float(tvbs_body_thrust_angle_est, lower_ang_limit, upper_ang_limit);

    // correct ahrs for body to rotor pitch offset
    ahrs_view->set_pitch_offset(-tvbs_body_thrust_angle_est);

    // limit the roll tilt demand so it does not modify the average rotor pitch
    float max_roll_tilt_available = MIN(fabsf(tvbs_body_thrust_angle_dem - lower_ang_limit),fabsf(tvbs_body_thrust_angle_dem - upper_ang_limit));
    float roll_tilt_demand = constrain_float(0.01f * aileron * tailsitter.vectored_hover_gain, -max_roll_tilt_available, max_roll_tilt_available);

    // modify left and right motor thrust angle to provide X axis control moment
    float thrust_angle_left  = tvbs_body_thrust_angle_dem + roll_tilt_demand;
    float thrust_angle_right = tvbs_body_thrust_angle_dem - roll_tilt_demand;

    // Constrain thrust angles to the physical movement range
    if (thrust_angle_left < lower_ang_limit) {
        tilt_saturated = true;
        thrust_angle_left = lower_ang_limit;
    } else if (thrust_angle_left > upper_ang_limit) {
        tilt_saturated = true;
        thrust_angle_left = upper_ang_limit;
    }
    if (thrust_angle_right < lower_ang_limit) {
        tilt_saturated = true;
        thrust_angle_right = lower_ang_limit;
    } else if (thrust_angle_right > upper_ang_limit) {
        tilt_saturated = true;
        thrust_angle_right = upper_ang_limit;
    }

    // Stop rate controller integrators winding up when angles are being constrained
    if (tilt_saturated) {
        motors->limit.roll_pitch = 1;
        motors->limit.yaw = 1;
    }

    // map to equivalent servo demands in centi-degrees
    float servo_angle_left; // angle of the left tilt servo in centi-degrees
    if (thrust_angle_left >= 0.0f) {
        // handle servo movement between trim and max pwm
        servo_angle_left = 4500.0f * thrust_angle_left / (float)tailsitter.tvbs_ang_max_deg;
    } else {
        // handle servo movement between trim and min pwm
        servo_angle_left = - 4500.0f * thrust_angle_left / (float)tailsitter.tvbs_ang_min_deg;
    }
    float servo_angle_right; // angle of the right tilt servo in centi-degrees
    if (thrust_angle_right >= 0.0f) {
        // handle servo movement between trim and max pwm
        servo_angle_right = 4500.0f * thrust_angle_right / (float)tailsitter.tvbs_ang_max_deg;
    } else {
        // handle servo movement between trim and min pwm
        servo_angle_right = - 4500.0f * thrust_angle_right / (float)tailsitter.tvbs_ang_min_deg;
    }

    // Output to tilt servo demands
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorLeft, servo_angle_left);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltMotorRight, servo_angle_right);

    /*
     * Calculate the elevator control surface deflection
    */

    // Calculate a filtered pitch acceleration for the wing in deg/sec^2
    float pitch_rate_deg = degrees(ahrs.get_gyro_latest().y);
    alpha = tvbs_dt_avg / MAX(tailsitter.tvbs_dtau_sec, tvbs_dt_avg);
    float state_prev = tvbs_pitch_rate_filt;
    tvbs_pitch_rate_filt = alpha * pitch_rate_deg + (1.0f - alpha) * tvbs_pitch_rate_filt;
    float pitch_rate_accel = (tvbs_pitch_rate_filt - state_prev) / tvbs_dt_avg;

    // Calculate a gain factor that reduces aerodynamic control gain when throttle rises above hover setting.
    // This allows for the control effectiveness increasing with (prop wash velocity)**2 and assuming that
    // propwash velocity is proportional to RPM and RPM is proportional to throttle.
    float reference_throttle = constrain_float(motors->get_throttle_hover(), 0.25f , 0.75f);
    float propwash_gain_factor = reference_throttle / constrain_float(motors->get_throttle() , reference_throttle , 1.0f);
    propwash_gain_factor *= propwash_gain_factor;

    // Calculate the thrust angle pitch rate.
    alpha = tvbs_dt_avg / MAX(tailsitter.tvbs_elev_hpf_tau_sec, tvbs_dt_avg);
    float rotor_pitch_rate;
    if (!control_loss_declared) {
        float derivative = (tvbs_body_thrust_angle_est - tvbs_body_thrust_angle_est_prev) / tvbs_dt_avg;
        tvbs_thrust_ang_deriv = alpha * derivative + (1.0f - alpha) * tvbs_thrust_ang_deriv;
        rotor_pitch_rate = pitch_rate_deg + tvbs_thrust_ang_deriv;
    } else {
        tvbs_thrust_ang_deriv = 0.0f;
        rotor_pitch_rate = 0.0f;
    }
    tvbs_body_thrust_angle_est_prev = tvbs_body_thrust_angle_est;

    // calculate an elevator correction using angular rate and acceleration feedback to control wing pitch angle
    float elev_demand = rotor_pitch_rate * tailsitter.tvbs_rotor_to_elev_gain * (constrain_float(motors->get_throttle() , reference_throttle , 1.0f) / reference_throttle)
            - pitch_rate_deg * tailsitter.tvbs_rate_gain
            - pitch_rate_accel * tailsitter.tvbs_dgain;
    float elev_trim_deg = 0.45f*tailsitter.tvbs_elev_trim_pcnt;

    /*
     * Calculate elevator and aileron gain factors that compensate for the increase in aero surface
     * effectiveness as the wing starts to increase speed, generates lift and rotates to a flight
     * orientation. Rate limit how fast the gain factor can reduce to help prevent coupling between gain
     * and wing pitch that can cause small amplitude pitch limit cycling.
    */
    float delta_min = -0.4f * tvbs_dt_avg; // gain factor is not allowed to reduce faster than 0.4/sec
    float sq_sine_wing_tilt = sq(sinf(ahrs_view->get_pitch_312_wing()));
    float raw_factor = constrain_float((1.0f - 0.01f * (float)tailsitter.tvbs_elev_gf * sq_sine_wing_tilt), 0.0f, 1.0f);
    if (raw_factor - _elev_gain_factor < delta_min) {
        _elev_gain_factor += delta_min;
    } else {
        _elev_gain_factor = constrain_float((1.0f - 0.01f * (float)tailsitter.tvbs_elev_gf * sq_sine_wing_tilt), 0.0f, 1.0f);
    }
    raw_factor = constrain_float((1.0f - 0.01f * (float)tailsitter.tvbs_ail_gf * sq_sine_wing_tilt), 0.0f, 1.0f);
    if (raw_factor - _ail_gain_factor < delta_min) {
        _ail_gain_factor += delta_min;
    } else {
        _ail_gain_factor = constrain_float((1.0f - 0.01f * (float)tailsitter.tvbs_elev_gf * sq_sine_wing_tilt), 0.0f, 1.0f);
    }

    // calculate an elevator used to prevent the wing being less tilted than the rotor at large tilt angles
    if (pitch_demand_rad < 0.0f &&
            pitch_312_mea_rad < 0.0f &&
            pitch_312_mea_rad > pitch_demand_rad) {
        elev_demand += degrees((pitch_demand_rad - pitch_312_mea_rad) * tailsitter.tvbs_wpe_gain * (-pitch_demand_rad/M_PI_2));
    } else if (pitch_demand_rad > 0.0f &&
               pitch_312_mea_rad > 0.0f &&
               pitch_312_mea_rad < pitch_demand_rad) {
        elev_demand += degrees((pitch_demand_rad - pitch_312_mea_rad) * tailsitter.tvbs_wpe_gain * (pitch_demand_rad/M_PI_2));
    }

    // sum all elevator contributions and limit to channel range
    elev_demand = constrain_float(elev_demand * propwash_gain_factor * _elev_gain_factor + elev_trim_deg, -45.0f, 45.0f);

    // Calculate the slew rate amplitude produced by the unmodified control feedback

    // calculate a low pass filtered elevator and obtain the derivative
    float elev_feedback_filtered = _elev_slew_rate_filter.apply(elev_demand, tvbs_dt_avg);
    float elev_slew_rate = (elev_feedback_filtered - _last_elev_feedback) / tvbs_dt_avg;
    _last_elev_feedback = elev_feedback_filtered;

    // rectify and filter
    alpha = tvbs_dt_avg / MAX(tailsitter.elev_slew_rate_tau_sec, tvbs_dt_avg);
    _elev_slew_rate_amplitude = alpha * fabsf(elev_slew_rate) + (1.0f - alpha) * _elev_slew_rate_amplitude;
    _elev_slew_rate_amplitude = fminf(_elev_slew_rate_amplitude, 10.0f * tailsitter.elev_slew_rate_max_dps);

    // Calculate the gain adjustment required to keep the servo slew rate within limits
    _limit_cycle_gain_modifier = tailsitter.elev_slew_rate_max_dps / fmaxf(_elev_slew_rate_amplitude, tailsitter.elev_slew_rate_max_dps);

    // Apply the gain adjustment and convert to centi-degrees
    float tvbs_elev_demand_cd = 100.0f * _limit_cycle_gain_modifier * elev_demand;

    // Output to elevator servo demand
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, tvbs_elev_demand_cd);

    /*
     * Calculate the aileron surface deflection using a combination of equivalent rudder
     * and aileron in the rotor frame projected onto the wing frame X-axis control moment.
    */
    float net_rotor_tilt_rad = radians(tvbs_body_thrust_angle_est); // filtered pitch angle of rotors relative to wing (rad)
    float wing_aileron_cd = propwash_gain_factor * _ail_gain_factor * (cosf(net_rotor_tilt_rad) * aileron + sinf(net_rotor_tilt_rad) * rudder);

    // Output to aileron servo demand
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, wing_aileron_cd);

    tvbs_active = true;

}
