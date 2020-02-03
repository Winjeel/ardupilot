#include "Plane.h"

void Plane::failsafe_short_on_event(enum FailsafeState fstype, mode_reason_t reason)
{
    if (g.fs_action_short == FS_ACTION_SHORT_DISABLED) {
        return;
    }

    Mode *failsafe_mode = control_mode;

    // This is how to handle a short loss of control signal failsafe.
    failsafe.state = fstype;
    failsafe.short_timer_ms = millis();
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event on: type=%u/reason=%u", fstype, reason);
    switch (control_mode->mode_number())
    {
        case Mode::Number::INITIALISING:
        case Mode::Number::TAKEOFF:
        case Mode::Number::CIRCLE:
        case Mode::Number::RTL:
            // ignored in these modes
            break;

        // stabilisation modes
        case Mode::Number::MANUAL:
        case Mode::Number::STABILIZE:
        case Mode::Number::TRAINING:
        case Mode::Number::ACRO:
        case Mode::Number::FLY_BY_WIRE_A:
        case Mode::Number::FLY_BY_WIRE_B:
        case Mode::Number::CRUISE:
        case Mode::Number::AUTOTUNE:
            if (g.fs_action_short == FS_ACTION_SHORT_FBWA) {
                failsafe_mode = &mode_fbwa;
            } else if (g.fs_action_short == FS_ACTION_SHORT_CIRCLE) {
                failsafe_mode = &mode_circle;
            } else if (g.fs_action_short == FS_ACTION_SHORT_BESTGUESS) {
                failsafe_mode = &mode_circle;
            } else if (g.fs_action_short == FS_ACTION_SHORT_LOITER) {
                failsafe_mode = &mode_loiter;
            } else {
                // unexpected action, so Circle
                gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe: Invalid short action! Defaulting to Circle mode.");
                failsafe_mode = &mode_circle;
            }
            break;

        case Mode::Number::QSTABILIZE:
        case Mode::Number::QHOVER:
        case Mode::Number::QLOITER:
        case Mode::Number::QLAND:
        case Mode::Number::QRTL:
        case Mode::Number::QAUTOTUNE:
        case Mode::Number::QACRO:
            failsafe_mode = &mode_qland;
        if (quadplane.options & QuadPlane::OPTION_FS_QRTL) {
            set_mode(mode_qrtl, reason);
        } else {
            set_mode(mode_qland, reason);
        }
            break;

        case Mode::Number::AUTO:
        case Mode::Number::LOITER:
        case Mode::Number::AVOID_ADSB:
        case Mode::Number::GUIDED:
            if (g.fs_action_short == FS_ACTION_SHORT_FBWA) {
                failsafe_mode = &mode_fbwa;
            } else if (g.fs_action_short == FS_ACTION_SHORT_CIRCLE) {
                failsafe_mode = &mode_circle;
            } else if (g.fs_action_short == FS_ACTION_SHORT_BESTGUESS) {
                // stay in current auto mode
            } else if (g.fs_action_short == FS_ACTION_SHORT_LOITER) {
                failsafe_mode = &mode_loiter;
            } else {
                // unexpected action, so stay in current auto mode
                gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe: Invalid short action! Remaining in current auto mode.");
            }
            break;
    }

    if (failsafe_mode != control_mode) {
        failsafe.saved_mode_number = control_mode->mode_number();
        failsafe.saved_mode_set = true;
        set_mode(*failsafe_mode, reason);
    }

    gcs().send_text(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode->mode_number());
}

void Plane::failsafe_long_on_event(enum FailsafeState fstype, mode_reason_t reason)
{
    // This is how to handle a long loss of control signal failsafe.
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Long event on: type=%u/reason=%u", fstype, reason);
    //  If the GCS is locked up we allow control to revert to RC
    RC_Channels::clear_overrides();
    failsafe.state = fstype;

    switch (control_mode->mode_number())
    {
        // stabilise modes
        case Mode::Number::MANUAL:
        case Mode::Number::CIRCLE:
        case Mode::Number::STABILIZE:
        case Mode::Number::TRAINING:
        case Mode::Number::ACRO:
        case Mode::Number::FLY_BY_WIRE_A:
        case Mode::Number::FLY_BY_WIRE_B:
        case Mode::Number::CRUISE:
        case Mode::Number::AUTOTUNE:
            if (g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if PARACHUTE == ENABLED
                parachute_release();
#endif
            } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
                set_mode(mode_fbwa, reason);
            } else if (g.fs_action_long == FS_ACTION_LONG_RTL) {
                set_mode(mode_rtl, reason);
            } else if (g.fs_action_long == FS_ACTION_LONG_CONTINUE) {
                set_mode(mode_rtl, reason);
            } else {
                // unexpected action, so RTL
                gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe: Invalid long action! Defaulting to RTL mode.");
                set_mode(mode_rtl, reason);
            }
            break;

        // auto modes
        case Mode::Number::AUTO:
        case Mode::Number::RTL:
        case Mode::Number::LOITER:
        case Mode::Number::AVOID_ADSB:
        case Mode::Number::GUIDED:
            if (g.fs_action_long == FS_ACTION_LONG_PARACHUTE) {
#if PARACHUTE == ENABLED
                parachute_release();
#endif
            } else if (g.fs_action_long == FS_ACTION_LONG_GLIDE) {
                set_mode(mode_fbwa, reason);
            } else if (g.fs_action_long == FS_ACTION_LONG_RTL) {
                set_mode(mode_rtl, reason);
            } else if (g.fs_action_long == FS_ACTION_LONG_CONTINUE) {
                // continue in the current auto mode
            } else {
                // unexpected action, so continue in the current auto mode
                gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe: Invalid long action! Defaulting to current mode.");
            }
            break;

        case Mode::Number::QSTABILIZE:
        case Mode::Number::QHOVER:
        case Mode::Number::QLOITER:
        case Mode::Number::QLAND:
        case Mode::Number::QRTL:
        case Mode::Number::QAUTOTUNE:
        case Mode::Number::QACRO:
            if (quadplane.options & QuadPlane::OPTION_FS_QRTL) {
                set_mode(mode_qrtl, reason);
            } else {
                set_mode(mode_qland, reason);
            }
            break;

        // if we're starting up, don't do anything
        case Mode::Number::INITIALISING:
        case Mode::Number::TAKEOFF:
            break;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "Flight mode = %u", (unsigned)control_mode->mode_number());
}

void Plane::failsafe_short_off_event(mode_reason_t reason)
{
    // We're back in radio contact
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Short event off: reason=%u", reason);
    failsafe.state = FailsafeState::None;

    // re-read the switch so we can return to our preferred mode
    // --------------------------------------------------------
    if (control_mode == &mode_circle && failsafe.saved_mode_set) {
        failsafe.saved_mode_set = false;
        set_mode_by_number(failsafe.saved_mode_number, reason);
    }
}

void Plane::failsafe_long_off_event(mode_reason_t reason)
{
    // We're back in radio contact
    gcs().send_text(MAV_SEVERITY_WARNING, "Failsafe. Long event off: reason=%u", reason);
    failsafe.state = FailsafeState::None;
}

void Plane::handle_battery_failsafe(const char *type_str, const int8_t action)
{
    switch ((Failsafe_Action)action) {
        case Failsafe_Action_QLand:
            if (quadplane.available()) {
                plane.set_mode(mode_qland, MODE_REASON_BATTERY_FAILSAFE);
                break;
            }
            FALLTHROUGH;
        case Failsafe_Action_Land:
            if (flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND && control_mode != &mode_qland) {
                // never stop a landing if we were already committed
                if (plane.mission.jump_to_landing_sequence()) {
                    plane.set_mode(mode_auto, MODE_REASON_BATTERY_FAILSAFE);
                    break;
                }
            }
            FALLTHROUGH;
        case Failsafe_Action_RTL:
            if (flight_stage != AP_Vehicle::FixedWing::FLIGHT_LAND && control_mode != &mode_qland && !quadplane.in_vtol_land_sequence()) {
                // never stop a landing if we were already committed
                set_mode(mode_rtl, MODE_REASON_BATTERY_FAILSAFE);
                aparm.throttle_cruise.load();
            }
            break;

        case Failsafe_Action_Terminate:
#if ADVANCED_FAILSAFE == ENABLED
            char battery_type_str[17];
            snprintf(battery_type_str, 17, "%s battery", type_str);
            afs.gcs_terminate(true, battery_type_str);
#else
            arming.disarm();
#endif
            break;

        case Failsafe_Action_Parachute:
#if PARACHUTE == ENABLED
            parachute_release();
#endif
            break;

        case Failsafe_Action_None:
            // don't actually do anything, however we should still flag the system as having hit a failsafe
            // and ensure all appropriate flags are going off to the user
            break;
    }
}
