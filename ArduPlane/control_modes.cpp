#include "Plane.h"

void Plane::read_control_switch()
{
    // custom handling of switch inputs for corvo controller
    if (quadplane.tailsitter.input_type == quadplane.TAILSITTER_CORVOX) {
        read_corvo_control_switch();
        return;
    }

    static bool switch_debouncer;
    uint8_t switchPosition = readSwitch();

    // If switchPosition = 255 this indicates that the mode control channel input was out of range
    // If we get this value we do not want to change modes.
    if(switchPosition == 255) return;

    if (failsafe.rc_failsafe || failsafe.throttle_counter > 0) {
        // when we are in rc_failsafe mode then RC input is not
        // working, and we need to ignore the mode switch channel
        return;
    }

    if (millis() - failsafe.last_valid_rc_ms > 100) {
        // only use signals that are less than 0.1s old.
        return;
    }

    // we look for changes in the switch position. If the
    // RST_SWITCH_CH parameter is set, then it is a switch that can be
    // used to force re-reading of the control switch. This is useful
    // when returning to the previous mode after a failsafe or fence
    // breach. This channel is best used on a momentary switch (such
    // as a spring loaded trainer switch).
    if (oldSwitchPosition != switchPosition ||
        (g.reset_switch_chan != 0 &&
         RC_Channels::get_radio_in(g.reset_switch_chan-1) > RESET_SWITCH_CHAN_PWM)) {

        if (switch_debouncer == false) {
            // this ensures that mode switches only happen if the
            // switch changes for 2 reads. This prevents momentary
            // spikes in the mode control channel from causing a mode
            // switch
            switch_debouncer = true;
            return;
        }

        set_mode((enum FlightMode)(flight_modes[switchPosition].get()), MODE_REASON_TX_COMMAND);

        oldSwitchPosition = switchPosition;
    }

    if (g.reset_mission_chan != 0 &&
        RC_Channels::get_radio_in(g.reset_mission_chan-1) > RESET_SWITCH_CHAN_PWM) {
        mission.start();
        prev_WP_loc = current_loc;
    }

    switch_debouncer = false;

#if PARACHUTE == ENABLED
    if (g.parachute_channel > 0) {
        if (RC_Channels::get_radio_in(g.parachute_channel-1) >= 1700) {
            parachute_manual_release();
        }
    }
#endif
    
#if HAVE_PX4_MIXER
    if (g.override_channel > 0) {
        // if the user has configured an override channel then check it
        bool override_requested = (RC_Channels::get_radio_in(g.override_channel-1) >= PX4IO_OVERRIDE_PWM);
        if (override_requested && !px4io_override_enabled) {
            if (hal.util->get_soft_armed() || (last_mixer_crc != -1)) {
                px4io_override_enabled = true;
                // disable output channels to force PX4IO override
                gcs().send_text(MAV_SEVERITY_WARNING, "PX4IO override enabled");
            } else {
                // we'll let the one second loop reconfigure the mixer. The
                // PX4IO code sometimes rejects a mixer, probably due to it
                // being busy in some way?
                gcs().send_text(MAV_SEVERITY_WARNING, "PX4IO override enable failed");
            }
        } else if (!override_requested && px4io_override_enabled) {
            px4io_override_enabled = false;
            SRV_Channels::enable_aux_servos();
            gcs().send_text(MAV_SEVERITY_WARNING, "PX4IO override disabled");
        }
        if (px4io_override_enabled && 
            hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_ARMED &&
            g.override_safety == 1) {
            // we force safety off, so that if this override is used
            // with a in-flight reboot it gives a way for the pilot to
            // re-arm and take manual control
            hal.rcout->force_safety_off();
        }
    }
#endif // HAVE_PX4_MIXER
}

uint8_t Plane::readSwitch(void)
{
    uint16_t pulsewidth = RC_Channels::get_radio_in(g.flight_mode_channel - 1);
    if (pulsewidth <= 900 || pulsewidth >= 2200) return 255;            // This is an error condition
    if (pulsewidth <= 1230) return 0;
    if (pulsewidth <= 1360) return 1;
    if (pulsewidth <= 1490) return 2;
    if (pulsewidth <= 1620) return 3;
    if (pulsewidth <= 1749) return 4;              // Software Manual
    return 5;                                                           // Hardware Manual
}

void Plane::read_corvo_control_switch()
{
    uint8_t changeMode = read_change_mode_select_switch();
    uint8_t controlSelect = read_control_select_switch();

    if (failsafe.rc_failsafe || failsafe.throttle_counter > 0) {
        // when we are in rc_failsafe mode then RC input is not
        // working, and we need to ignore the mode switch channel
        return;
    }

    if (millis() - failsafe.last_valid_rc_ms > 100) {
        // only use signals that are less than 0.1s old.
        return;
    }

    // Use the 'Change Mode Select' button to toggle between QLOITER and CRUISE control modes

    // increment/decrement counter based on switch position.
    if ((changeMode == 1) && (changeModeCount < 5)) {
        changeModeCount++;
    } else if ((changeMode == 0) && (changeModeCount >= 1)){
        changeModeCount--;
    }

    if (changeModeCount == 5 && !oldChangeMode) {
        // switch press confirmed
        oldChangeMode = true;
        if (quadplane.in_vtol_mode()) {
            // in VTOL mode so change to FW CRUISE
            set_mode(CRUISE, MODE_REASON_TX_COMMAND);
        } else {
            // in FW mode so change to VTOL QLOITER
            set_mode(QLOITER, MODE_REASON_TX_COMMAND);
        }
        // disable stick control of the payload mount and reset the LOS elevation to MNT_INIT_ELEV
        vtolCameraControlMode = false;
        camera_mount.set_elev_park(true);
        camera_mount.reset_elev();
    } else if (changeModeCount == 0 && oldChangeMode) {
        // switch release confirmed
        oldChangeMode = false;
    }

    // When in FW operation use the 'Control Select' button to toggle between CRUISE (vehicle control) and GUIDED (camera control) modes
    // When in default VTOL mode QLOITER, toggle the 'vtolCameraControlMode' flag which casues roll/pitch stick inputs to switch between
    // control of positon and control of camera pan/elevation.

    // increment/decrement counter based on switch position.
    if ((controlSelect == 1) && (controlSelectCount < 5)) {
        controlSelectCount++;
    } else if ((controlSelect == 0) && (controlSelectCount >= 1)){
        controlSelectCount--;
    }

    bool toggle_fw_flight_mode = false;
    bool resetVtolCameraControl = false;
    if (controlSelectCount == 5 && !oldControlSelect) {
        // switch press confirmed
        oldControlSelect = true;
        controlSelectTime_ms = millis();
        if (!quadplane.in_vtol_mode()) {
            toggle_fw_flight_mode = true;
        } else if (control_mode == QLOITER) {
            // in QLOITER we transfer operator control between positon to camera
            vtolCameraControlMode = !vtolCameraControlMode;
            resetVtolCameraControl = true;
        } else {
            // we don't do camera control in other VTOL modes
            vtolCameraControlMode = false;
            resetVtolCameraControl = true;
        }
    } else if (controlSelectCount == 0 && oldControlSelect) {
        // switch release confirmed
        oldControlSelect = false;
        // handle case where the operator has released the 'Control Select' button before the 1 second latch timeout
        if ((millis() - controlSelectTime_ms) < 1000) {
            if (!quadplane.in_vtol_mode()) {
                toggle_fw_flight_mode = true;
            } else if (control_mode == QLOITER) {
                // in QLOITER we transfer operator control between positon to camera
                vtolCameraControlMode = !vtolCameraControlMode;
                resetVtolCameraControl = true;
            } else {
                // we don't do camera control in other VTOL modes
                vtolCameraControlMode = false;
                resetVtolCameraControl = true;
            }
        }
    }

    // switch camera mount mode control
    if (resetVtolCameraControl) {
        if (vtolCameraControlMode) {
            // camera yaw/elevation pointing is controlled by the roll/pitch stick and vehicle holds at previous horizontal position
            camera_mount.set_elev_park(false);
            camera_mount.set_yaw_target(degrees(quadplane.ahrs_view->yaw));
        } else  {
            // yaw moves with vehicle
            camera_mount.set_elev_park(true);
        }
    }

    // perform the FW mode change between between vehicle control and camera control
    if (toggle_fw_flight_mode && !quadplane.in_vtol_mode()) {
        if (control_mode != GUIDED) {
            // switch to camera control mode
            set_mode(GUIDED, MODE_REASON_TX_COMMAND);
        } else {
            // we are in the camera control mode so switch to default vehicle control mode
            set_mode(CRUISE, MODE_REASON_TX_COMMAND);
            camera_mount.set_mode(MAV_MOUNT_MODE_NEUTRAL);
        }
     }
}

uint8_t Plane::read_change_mode_select_switch(void)
{
    uint16_t pulsewidth = RC_Channels::get_radio_in(5);
    uint8_t ret = 255;
    if (pulsewidth <= 900 || pulsewidth >= 2200) {
        ret = 255;            // This is an error condition
    } else if (pulsewidth <= 1500) {
        ret = 0;
    } else {
        ret = 1;
    }
    return ret;
}

uint8_t Plane::read_control_select_switch(void)
{
    uint16_t pulsewidth = RC_Channels::get_radio_in(4);
    uint8_t ret = 255;
    if (pulsewidth <= 900 || pulsewidth >= 2200) {
        ret = 255;            // This is an error condition
    } else if (pulsewidth <= 1500) {
        ret = 0;
    } else {
        ret = 1;
    }
    return ret;
}

void Plane::reset_control_switch()
{
    oldSwitchPosition = 254;
    read_control_switch();

    oldControlSelect = false;
    controlSelectCount = 0;
    vtolCameraControlMode = false;
    controlSelectTime_ms = 0;
    read_control_select_switch();

    oldChangeMode = false;
    changeModeCount = 0;
    read_change_mode_select_switch();

    if (quadplane.tailsitter.input_type == quadplane.TAILSITTER_CORVOX) {
        // start corvo in QLOITER mode
        previous_mode = control_mode = QLOITER;

        // Run payload pointing in park mode where yaw follows vehicle and elevation is set to value set by MNT_INIT_ELEV parmeter
        camera_mount.set_elev_park(true);
        camera_mount.reset_elev();
    }
}

/*
  called when entering autotune
 */
void Plane::autotune_start(void)
{
    rollController.autotune_start();
    pitchController.autotune_start();
}

/*
  called when exiting autotune
 */
void Plane::autotune_restore(void)
{
    rollController.autotune_restore();
    pitchController.autotune_restore();
}

/*
  enable/disable autotune for AUTO modes
 */
void Plane::autotune_enable(bool enable)
{
    if (enable) {
        autotune_start();
    } else {
        autotune_restore();
    }
}

/*
  are we flying inverted?
 */
bool Plane::fly_inverted(void)
{
    if (control_mode == MANUAL) {
        return false;
    }
    if (inverted_flight) {
        // controlled with aux switch
        return true;
    }
    if (control_mode == AUTO && auto_state.inverted_flight) {
        return true;
    }
    return false;
}
