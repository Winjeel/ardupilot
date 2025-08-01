#pragma once

#include "Copter.h"
#include <AP_Math/chirp.h>
#include <AP_ExternalControl/AP_ExternalControl_config.h> // TODO why is this needed if Copter.h includes this
#include <AP_HAL/Semaphores.h>

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
#include "afs_copter.h"
#endif

class Parameters;
class ParametersG2;

class GCS_Copter;

// object shared by both Guided and Auto for takeoff.
// position controller controls vehicle but the user can control the yaw.
class _AutoTakeoff {
public:
    void run();
    void start(float complete_alt_cm, bool terrain_alt);
    bool get_completion_pos(Vector3p& pos_neu_cm);

    bool complete;          // true when takeoff is complete

private:
    // altitude above-ekf-origin below which auto takeoff does not control horizontal position
    bool no_nav_active;
    float no_nav_alt_cm;

    // auto takeoff variables
    float complete_alt_cm;  // completion altitude expressed in cm above ekf origin or above terrain (depending upon auto_takeoff_terrain_alt)
    bool terrain_alt;       // true if altitudes are above terrain
    Vector3p complete_pos;  // target takeoff position as offset from ekf origin in cm
};

#if AC_PAYLOAD_PLACE_ENABLED
class PayloadPlace {
public:
    void run();
    void start_descent();
    bool verify();

    enum class State : uint8_t {
        FlyToLocation,
        Descent_Start,
        Descent,
        Release,
        Releasing,
        Delay,
        Ascent_Start,
        Ascent,
        Done,
    };

    // these are set by the Mission code:
    State state = State::Descent_Start; // records state of payload place
    float descent_max_cm;

private:

    uint32_t descent_established_time_ms; // milliseconds
    uint32_t place_start_time_ms; // milliseconds
    float descent_thrust_level;
    float descent_start_altitude_cm;
    float descent_speed_cms;
};
#endif

class Mode {
    friend class PayloadPlace;

public:

    // Auto Pilot Modes enumeration
    enum class Number : uint8_t {
        STABILIZE =     0,  // manual airframe angle with manual throttle
        ACRO =          1,  // manual body-frame angular rate with manual throttle
        ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        AUTO =          3,  // fully automatic waypoint control using mission commands
        GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        RTL =           6,  // automatic return to launching point
        CIRCLE =        7,  // automatic circular flight with automatic throttle
        LAND =          9,  // automatic landing with horizontal position control
        DRIFT =        11,  // semi-autonomous position, yaw and throttle control
        SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        FLIP =         14,  // automatically flip the vehicle on the roll axis
        AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
        FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
        FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
        ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
        SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
        AUTOROTATE =   26,  // Autonomous autorotation
        AUTO_RTL =     27,  // Auto RTL, this is not a true mode, AUTO will report as this mode if entered to perform a DO_LAND_START Landing sequence
        TURTLE =       28,  // Flip over after crash

        // Mode number 30 reserved for "offboard" for external/lua control.

        // Mode number 127 reserved for the "drone show mode" in the Skybrush
        // fork at https://github.com/skybrush-io/ardupilot
    };

    // constructor
    Mode(void);

    // do not allow copying
    CLASS_NO_COPY(Mode);

    friend class _AutoTakeoff;

    // returns a unique number specific to this mode
    virtual Number mode_number() const = 0;

    // child classes should override these methods
    virtual bool init(bool ignore_checks) {
        return true;
    }
    virtual void exit() {};
    virtual void run() = 0;
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(AP_Arming::Method method) const = 0;
    virtual bool is_autopilot() const = 0;
    virtual bool has_user_takeoff(bool must_navigate) const { return false; }
    virtual bool in_guided_mode() const { return false; }
    virtual bool logs_attitude() const { return false; }
    virtual bool allows_save_trim() const { return false; }
    virtual bool allows_auto_trim() const { return false; }
    virtual bool allows_autotune() const { return false; }
    virtual bool allows_flip() const { return false; }
    virtual bool crash_check_enabled() const { return true; }

    // "no pilot input" here means eg. in RC failsafe
    virtual bool allows_entry_in_rc_failsafe() const { return true; }

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Return the type of this mode for use by advanced failsafe
    virtual AP_AdvancedFailsafe_Copter::control_mode afs_mode() const { return AP_AdvancedFailsafe_Copter::control_mode::AFS_STABILIZED; }
#endif

    // Return true if the throttle high arming check can be skipped when arming from GCS or Scripting
    virtual bool allows_GCS_or_SCR_arming_with_throttle_high() const { return false; }

#if FRAME_CONFIG == HELI_FRAME
    virtual bool allows_inverted() const { return false; };
#endif

    // return a string for this flightmode
    virtual const char *name() const = 0;
    virtual const char *name4() const = 0;

    bool do_user_takeoff(float takeoff_alt_cm, bool must_navigate);
    virtual bool is_taking_off() const;
    static void takeoff_stop() { takeoff.stop(); }

    virtual bool is_landing() const { return false; }

    // mode requires terrain to be present to be functional
    virtual bool requires_terrain_failsafe() const { return false; }

    // functions for reporting to GCS
    virtual bool get_wp(Location &loc) const { return false; };
    virtual float wp_bearing_deg() const { return 0; }
    virtual float wp_distance_m() const { return 0.0f; }
    virtual float crosstrack_error_m() const { return 0.0f;}

    // functions to support MAV_CMD_DO_CHANGE_SPEED
    virtual bool set_speed_xy_cms(float speed_xy_cms) {return false;}
    virtual bool set_speed_up_cms(float speed_xy_cms) {return false;}
    virtual bool set_speed_down_cms(float speed_xy_cms) {return false;}

    virtual int32_t get_alt_above_ground_cm(void) const;

    // pilot input processing
    void get_pilot_desired_lean_angles_rad(float &roll_out_rad, float &pitch_out_rad, float angle_max_rad, float angle_limit_rad) const;
    float get_pilot_desired_yaw_rate_rads() const;
    Vector2f get_pilot_desired_velocity(float vel_max) const;
    float get_pilot_desired_throttle() const;

    // returns climb target_rate reduced to avoid obstacles and
    // altitude fence
    float get_avoidance_adjusted_climbrate_cms(float target_rate_cms);

    // send output to the motors, can be overridden by subclasses
    virtual void output_to_motors();

    // returns true if pilot's yaw input should be used to adjust vehicle's heading
    virtual bool use_pilot_yaw() const {return true; }

    // pause and resume a mode
    virtual bool pause() { return false; };
    virtual bool resume() { return false; };

    // handle situations where the vehicle is on the ground waiting for takeoff
    void make_safe_ground_handling(bool force_throttle_unlimited = false);

    // true if weathervaning is allowed in the current mode
#if WEATHERVANE_ENABLED
    virtual bool allows_weathervaning() const { return false; }
#endif

protected:

    // helper functions
    bool is_disarmed_or_landed() const;
    void zero_throttle_and_relax_ac(bool spool_up = false);
    void zero_throttle_and_hold_attitude();

    // Return stopping point as a location with above origin alt frame
    Location get_stopping_point() const;

    // functions to control normal landing.  pause_descent is true if vehicle should not descend
    void land_run_horizontal_control();
    void land_run_vertical_control(bool pause_descent = false);
    void land_run_horiz_and_vert_control(bool pause_descent = false) {
        land_run_horizontal_control();
        land_run_vertical_control(pause_descent);
    }

#if AC_PAYLOAD_PLACE_ENABLED
    // payload place flight behaviour:
    static PayloadPlace payload_place;
#endif

    // run normal or precision landing (if enabled)
    // pause_descent is true if vehicle should not descend
    void land_run_normal_or_precland(bool pause_descent = false);

#if AC_PRECLAND_ENABLED
    // Go towards a position commanded by prec land state machine in order to retry landing
    // The passed in location is expected to be NED and in meters
    void precland_retry_position(const Vector3f &retry_pos);

    // Run precland statemachine. This function should be called from any mode that wants to do precision landing.
    // This handles everything from prec landing, to prec landing failures, to retries and failsafe measures
    void precland_run();
#endif

    // return expected input throttle setting to hover:
    virtual float throttle_hover() const;

    // Alt_Hold based flight mode states used in Alt_Hold, Loiter, and Sport
    enum class AltHoldModeState {
        MotorStopped,
        Takeoff,
        Landed_Ground_Idle,
        Landed_Pre_Takeoff,
        Flying
    };
    AltHoldModeState get_alt_hold_state(float target_climb_rate_cms);

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AC_WPNav *&wp_nav;
    AC_Loiter *&loiter_nav;
    AC_PosControl *&pos_control;
    AP_AHRS &ahrs;
    AC_AttitudeControl *&attitude_control;
    MOTOR_CLASS *&motors;
    RC_Channel *&channel_roll;
    RC_Channel *&channel_pitch;
    RC_Channel *&channel_throttle;
    RC_Channel *&channel_yaw;
    float &G_Dt;

    // note that we support two entirely different automatic takeoffs:

    // "user-takeoff", which is available in modes such as ALT_HOLD
    // (see has_user_takeoff method).  "user-takeoff" is a simple
    // reach-altitude-based-on-pilot-input-or-parameter routine.

    // "auto-takeoff" is used by both Guided and Auto, and is
    // basically waypoint navigation with pilot yaw permitted.

    // user-takeoff support; takeoff state is shared across all mode instances
    class _TakeOff {
    public:
        void start(float alt_cm);
        void stop();
        void do_pilot_takeoff(float& pilot_climb_rate);
        bool triggered(float target_climb_rate) const;

        bool running() const { return _running; }
    private:
        bool _running;
        float take_off_start_alt;
        float take_off_complete_alt;
    };

    static _TakeOff takeoff;

    virtual bool do_user_takeoff_start(float takeoff_alt_cm);

    static _AutoTakeoff auto_takeoff;

public:
    // Navigation Yaw control
    class AutoYaw {

    public:

        // Autopilot Yaw Mode enumeration
        enum class Mode {
            HOLD =             0,   // hold zero yaw rate
            LOOK_AT_NEXT_WP =  1,   // point towards next waypoint (no pilot input accepted)
            ROI =              2,   // point towards a location held in roi (no pilot input accepted)
            FIXED =            3,   // point towards a particular angle (no pilot input accepted)
            LOOK_AHEAD =       4,   // point in the direction the copter is moving
            RESET_TO_ARMED_YAW = 5, // point towards heading at time motors were armed
            ANGLE_RATE =       6,   // turn at a specified rate from a starting angle
            RATE =             7,   // turn at a specified rate (held in auto_yaw_rate)
            CIRCLE =           8,   // use AC_Circle's provided yaw (used during Loiter-Turns commands)
            PILOT_RATE =       9,   // target rate from pilot stick
            WEATHERVANE =     10,   // yaw into wind
        };

        // mode(): current method of determining desired yaw:
        Mode mode() const { return _mode; }
        void set_mode_to_default(bool rtl);
        void set_mode(Mode new_mode);
        Mode default_mode(bool rtl) const;

        void set_rate_rad(float turn_rate_rads);

        // set_roi(...): set a "look at" location:
        void set_roi(const Location &roi_location);

        void set_fixed_yaw_rad(float angle_rad,
                               float turn_rate_rads,
                               int8_t direction,
                               bool relative_angle);

        void set_yaw_angle_and_rate_rad(float yaw_angle_rad, float yaw_rate_rads);

        void set_yaw_angle_offset_deg(const float yaw_angle_offset_deg);

        bool reached_fixed_yaw_target();

#if WEATHERVANE_ENABLED
        void update_weathervane(const float pilot_yaw_rads);
#endif

        AC_AttitudeControl::HeadingCommand get_heading();

    private:

        // yaw_rad(): main product of AutoYaw; the heading:
        float yaw_rad();

        // rate_rads(): desired yaw rate in centidegrees/second:
        float rate_rads();

        // Returns the yaw angle (in radians) representing the direction of horizontal motion.
        float look_ahead_yaw_rad();

        float roi_yaw_rad() const;

        // auto flight mode's yaw mode
        Mode _mode = Mode::LOOK_AT_NEXT_WP;
        Mode _last_mode;

        // Yaw will point at this location if mode is set to Mode::ROI
        Vector3f roi;

        // yaw used for YAW_FIXED yaw_mode
        float _fixed_yaw_offset_rad;

        // Radians/s we should turn
        float _fixed_yaw_slewrate_rads;

        // time of the last yaw update
        uint32_t _last_update_ms;

        // heading when in yaw_look_ahead_yaw
        float _look_ahead_yaw_rad;

        // turn heading (rad) and rate (rads) when auto_yaw_mode is set to AUTO_YAW_RATE
        float _yaw_angle_rad;
        float _yaw_rate_rads;
        float _pilot_yaw_rate_rads;
    };
    static AutoYaw auto_yaw;

    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the Mode base
    // class.
    float get_pilot_desired_climb_rate();
    float get_non_takeoff_throttle(void);
    void update_simple_mode(void);
    bool set_mode(Mode::Number mode, ModeReason reason);
    void set_land_complete(bool b);
    GCS_Copter &gcs();
    uint16_t get_pilot_speed_dn(void);
    // end pass-through functions
};


#if MODE_ACRO_ENABLED
class ModeAcro : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::ACRO; }

    enum class Trainer {
        OFF = 0,
        LEVELING = 1,
        LIMITED = 2,
    };

    enum class AcroOptions {
        AIR_MODE = 1 << 0,
        RATE_LOOP_ONLY = 1 << 1,
    };

    virtual void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool init(bool ignore_checks) override;
    void exit() override;
    // Called when air mode is enabled via AUX switch; prevents automatic reset to default air_mode state
    void air_mode_aux_changed();
    bool allows_save_trim() const override { return true; }
    bool allows_flip() const override { return true; }
    bool crash_check_enabled() const override { return false; }
    bool allows_entry_in_rc_failsafe() const override { return false; }

protected:

    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    // get_pilot_desired_rates_rads - transform pilot's normalised roll pitch and yaw input into a desired lean angle rates
    // the function returns desired angle rates in radians-per-second
    void get_pilot_desired_rates_rads(float &roll_out_rads, float &pitch_out_rads, float &yaw_out_rads);

    float throttle_hover() const override;

private:
    bool disable_air_mode_reset;
};
#endif

#if FRAME_CONFIG == HELI_FRAME
class ModeAcro_Heli : public ModeAcro {

public:
    // inherit constructor
    using ModeAcro::Mode;

    bool init(bool ignore_checks) override;
    void run() override;
    void virtual_flybar( float &roll_out, float &pitch_out, float &yaw_out, float pitch_leak, float roll_leak);

protected:
private:
};
#endif


class ModeAltHold : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::ALT_HOLD; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }
    bool allows_autotune() const override { return true; }
    bool allows_flip() const override { return true; }
    bool allows_auto_trim() const override { return true; }
    bool allows_save_trim() const override { return true; }
#if FRAME_CONFIG == HELI_FRAME
    bool allows_inverted() const override { return true; };
#endif
protected:

    const char *name() const override { return "ALT_HOLD"; }
    const char *name4() const override { return "ALTH"; }

private:

};

class ModeAuto : public Mode {

public:
    friend class PayloadPlace;  // in case wp_run is accidentally required

    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return auto_RTL? Number::AUTO_RTL : Number::AUTO; }

    bool init(bool ignore_checks) override;
    void exit() override;
    void run() override;

    bool requires_GPS() const override;
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override;
    bool is_autopilot() const override { return true; }
    bool in_guided_mode() const override { return _mode == SubMode::NAVGUIDED || _mode == SubMode::NAV_SCRIPT_TIME; }
#if FRAME_CONFIG == HELI_FRAME
    bool allows_inverted() const override { return true; };
#endif

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Return the type of this mode for use by advanced failsafe
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    // Return true if the throttle high arming check can be skipped when arming from GCS or Scripting
    bool allows_GCS_or_SCR_arming_with_throttle_high() const override { return true; }

    // Auto modes
    enum class SubMode : uint8_t {
        TAKEOFF,
        WP,
        LAND,
        RTL,
        CIRCLE_MOVE_TO_EDGE,
        CIRCLE,
        NAVGUIDED,
        LOITER,
        LOITER_TO_ALT,
#if AP_MISSION_NAV_PAYLOAD_PLACE_ENABLED && AC_PAYLOAD_PLACE_ENABLED
        NAV_PAYLOAD_PLACE,
#endif
        NAV_SCRIPT_TIME,
        NAV_ATTITUDE_TIME,
    };

    // set submode.  returns true on success, false on failure
    void set_submode(SubMode new_submode);

    // pause continue in auto mode
    bool pause() override;
    bool resume() override;
    bool paused() const;

    bool loiter_start();
    void rtl_start();
    void takeoff_start(const Location& dest_loc);
    bool wp_start(const Location& dest_loc);
    void land_start();
    void circle_movetoedge_start(const Location &circle_center, float radius_m, bool ccw_turn);
    void circle_start();
    void nav_guided_start();

    bool is_landing() const override;

    bool is_taking_off() const override;
    bool use_pilot_yaw() const override;

    bool set_speed_xy_cms(float speed_xy_cms) override;
    bool set_speed_up_cms(float speed_up_cms) override;
    bool set_speed_down_cms(float speed_down_cms) override;

    bool requires_terrain_failsafe() const override { return true; }

    void payload_place_start();

    // for GCS_MAVLink to call:
    bool do_guided(const AP_Mission::Mission_Command& cmd);

    // Go straight to landing sequence via DO_LAND_START, if succeeds pretend to be Auto RTL mode
    bool jump_to_landing_sequence_auto_RTL(ModeReason reason);

    // Join mission after DO_RETURN_PATH_START waypoint, if succeeds pretend to be Auto RTL mode
    bool return_path_start_auto_RTL(ModeReason reason);

    // Try join return path else do land start
    bool return_path_or_jump_to_landing_sequence_auto_RTL(ModeReason reason);

    // lua accessors for nav script time support
    bool nav_script_time(uint16_t &id, uint8_t &cmd, float &arg1, float &arg2, int16_t &arg3, int16_t &arg4);
    void nav_script_time_done(uint16_t id);

    AP_Mission mission{
        FUNCTOR_BIND_MEMBER(&ModeAuto::start_command, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&ModeAuto::verify_command, bool, const AP_Mission::Mission_Command &),
        FUNCTOR_BIND_MEMBER(&ModeAuto::exit_mission, void)};

    // Mission change detector
    AP_Mission_ChangeDetector mis_change_detector;

    // true if weathervaning is allowed in auto
#if WEATHERVANE_ENABLED
    bool allows_weathervaning(void) const override;
#endif

    // Get height above ground, uses landing height if available
    int32_t get_alt_above_ground_cm() const override;

protected:

    const char *name() const override { return auto_RTL? "AUTO RTL" : "AUTO"; }
    const char *name4() const override { return auto_RTL? "ARTL" : "AUTO"; }

    float wp_distance_m() const override;
    float wp_bearing_deg() const override;
    float crosstrack_error_m() const override { return wp_nav->crosstrack_error_m();}
    bool get_wp(Location &loc) const override;

private:

    enum class Option : int32_t {
        AllowArming                        = (1 << 0U),
        AllowTakeOffWithoutRaisingThrottle = (1 << 1U),
        IgnorePilotYaw                     = (1 << 2U),
        AllowWeatherVaning                 = (1 << 7U),
    };
    bool option_is_enabled(Option option) const;

    // Enter auto rtl pseudo mode
    bool enter_auto_rtl(ModeReason reason);

    bool start_command(const AP_Mission::Mission_Command& cmd);
    bool verify_command(const AP_Mission::Mission_Command& cmd);
    void exit_mission();

    bool check_for_mission_change();    // detect external changes to mission

    void takeoff_run();
    void wp_run();
    void land_run();
    void rtl_run();
    void circle_run();
    void nav_guided_run();
    void loiter_run();
    void loiter_to_alt_run();
    void nav_attitude_time_run();

    // return the Location portion of a command.  If the command's lat and lon and/or alt are zero the default_loc's lat,lon and/or alt are returned instead
    Location loc_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc) const;

    SubMode _mode = SubMode::TAKEOFF;   // controls which auto controller is run

    bool shift_alt_to_current_alt(Location& target_loc) const;

    // subtract position controller offsets from target location
    // should be used when the location will be used as a target for the position controller
    void subtract_pos_offsets(Location& target_loc) const;

    void do_takeoff(const AP_Mission::Mission_Command& cmd);
    void do_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool set_next_wp(const AP_Mission::Mission_Command& current_cmd, const Location &default_loc);
    void do_land(const AP_Mission::Mission_Command& cmd);
    void do_loiter_unlimited(const AP_Mission::Mission_Command& cmd);
    void do_circle(const AP_Mission::Mission_Command& cmd);
    void do_loiter_time(const AP_Mission::Mission_Command& cmd);
    void do_loiter_to_alt(const AP_Mission::Mission_Command& cmd);
    void do_spline_wp(const AP_Mission::Mission_Command& cmd);
    void get_spline_from_cmd(const AP_Mission::Mission_Command& cmd, const Location& default_loc, Location& dest_loc, Location& next_dest_loc, bool& next_dest_loc_is_spline);
#if AC_NAV_GUIDED
    void do_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
    void do_guided_limits(const AP_Mission::Mission_Command& cmd);
#endif
    void do_nav_delay(const AP_Mission::Mission_Command& cmd);
    void do_wait_delay(const AP_Mission::Mission_Command& cmd);
    void do_within_distance(const AP_Mission::Mission_Command& cmd);
    void do_yaw(const AP_Mission::Mission_Command& cmd);
    void do_change_speed(const AP_Mission::Mission_Command& cmd);
    void do_set_home(const AP_Mission::Mission_Command& cmd);
    void do_roi(const AP_Mission::Mission_Command& cmd);
    void do_mount_control(const AP_Mission::Mission_Command& cmd);
#if HAL_PARACHUTE_ENABLED
    void do_parachute(const AP_Mission::Mission_Command& cmd);
#endif
#if AP_WINCH_ENABLED
    void do_winch(const AP_Mission::Mission_Command& cmd);
#endif
    void do_payload_place(const AP_Mission::Mission_Command& cmd);
    void do_RTL(void);
#if AP_SCRIPTING_ENABLED
    void do_nav_script_time(const AP_Mission::Mission_Command& cmd);
#endif
    void do_nav_attitude_time(const AP_Mission::Mission_Command& cmd);

    bool verify_takeoff();
    bool verify_land();
    bool verify_payload_place();
    bool verify_loiter_unlimited();
    bool verify_loiter_time(const AP_Mission::Mission_Command& cmd);
    bool verify_loiter_to_alt() const;
    bool verify_RTL();
    bool verify_wait_delay();
    bool verify_within_distance();
    bool verify_yaw();
    bool verify_nav_wp(const AP_Mission::Mission_Command& cmd);
    bool verify_circle(const AP_Mission::Mission_Command& cmd);
    bool verify_spline_wp(const AP_Mission::Mission_Command& cmd);
#if AC_NAV_GUIDED
    bool verify_nav_guided_enable(const AP_Mission::Mission_Command& cmd);
#endif
    bool verify_nav_delay(const AP_Mission::Mission_Command& cmd);
#if AP_SCRIPTING_ENABLED
    bool verify_nav_script_time();
#endif
    bool verify_nav_attitude_time(const AP_Mission::Mission_Command& cmd);

    // Loiter control
    uint16_t loiter_time_max;                // How long we should stay in Loiter Mode for mission scripting (time in seconds)
    uint32_t loiter_time;                    // How long have we been loitering - The start time in millis

    struct {
        bool reached_destination_xy : 1;
        bool loiter_start_done : 1;
        bool reached_alt : 1;
        float alt_error_cm;
        int32_t alt;
    } loiter_to_alt;

    // Delay the next navigation command
    uint32_t nav_delay_time_max_ms;  // used for delaying the navigation commands (eg land,takeoff etc.)
    uint32_t nav_delay_time_start_ms;

    // Delay Mission Scripting Command
    int32_t condition_value;  // used in condition commands (eg delay, change alt, etc.)
    uint32_t condition_start;

    // Land within Auto state
    enum class State {
        FlyToLocation = 0,
        Descending = 1
    };
    State state = State::FlyToLocation;

    bool waiting_to_start;  // true if waiting for vehicle to be armed or EKF origin before starting mission

    // True if we have entered AUTO to perform a DO_LAND_START landing sequence and we should report as AUTO RTL mode
    bool auto_RTL;

#if AP_SCRIPTING_ENABLED
    // nav_script_time command variables
    struct {
        bool done;          // true once lua script indicates it has completed
        uint16_t id;        // unique id to avoid race conditions between commands and lua scripts
        uint32_t start_ms;  // system time nav_script_time command was received (used for timeout)
        uint8_t command;    // command number provided by mission command
        uint8_t timeout_s;  // timeout (in seconds) provided by mission command
        float arg1;         // 1st argument provided by mission command
        float arg2;         // 2nd argument provided by mission command
        int16_t arg3;       // 3rd argument provided by mission command
        int16_t arg4;       // 4th argument provided by mission command
    } nav_scripting;
#endif

    // nav attitude time command variables
    struct {
        int16_t roll_deg;   // target roll angle in degrees.  provided by mission command
        int8_t pitch_deg;   // target pitch angle in degrees.  provided by mission command
        int16_t yaw_deg;    // target yaw angle in degrees.  provided by mission command
        float climb_rate;   // climb rate in m/s. provided by mission command
        uint32_t start_ms;  // system time that nav attitude time command was received (used for timeout)
    } nav_attitude_time;

    // desired speeds
    struct {
        float xy;     // desired speed horizontally in m/s. 0 if unset
        float up;     // desired speed upwards in m/s. 0 if unset
        float down;   // desired speed downwards in m/s. 0 if unset
    } desired_speed_override;

    float circle_last_num_complete;
};

#if AUTOTUNE_ENABLED
/*
  wrapper class for AC_AutoTune
 */

#if FRAME_CONFIG == HELI_FRAME
class AutoTune : public AC_AutoTune_Heli
#else
class AutoTune : public AC_AutoTune_Multi
#endif
{
public:
    bool init() override;
    void run() override;

protected:
    bool position_ok() override;
    float get_pilot_desired_climb_rate_cms(void) const override;
    void get_pilot_desired_rp_yrate_rad(float &des_roll_rad, float &des_pitch_rad, float &des_yaw_rate_rads) override;
    void init_z_limits() override;
#if HAL_LOGGING_ENABLED
    void log_pids() override;
#endif
};

class ModeAutoTune : public Mode {

    // ParametersG2 sets a pointer within our autotune object:
    friend class ParametersG2;

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::AUTOTUNE; }

    bool init(bool ignore_checks) override;
    void exit() override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    bool is_autopilot() const override { return false; }

    AutoTune autotune;

protected:

    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }
};
#endif


class ModeBrake : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::BRAKE; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return true; }

    void timeout_to_loiter_ms(uint32_t timeout_ms);

protected:

    const char *name() const override { return "BRAKE"; }
    const char *name4() const override { return "BRAK"; }

private:

    uint32_t _timeout_start;
    uint32_t _timeout_ms;

};


class ModeCircle : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::CIRCLE; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

    float wp_distance_m() const override;
    float wp_bearing_deg() const override;

private:

    // Circle
    bool speed_changing = false;     // true when the roll stick is being held to facilitate stopping at 0 rate
};


class ModeDrift : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::DRIFT; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool allows_entry_in_rc_failsafe() const override { return false; }

protected:

    const char *name() const override { return "DRIFT"; }
    const char *name4() const override { return "DRIF"; }

private:

    float get_throttle_assist(float velz, float pilot_throttle_scaled);

};


class ModeFlip : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::FLIP; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return false; }
    bool crash_check_enabled() const override { return false; }

protected:

    const char *name() const override { return "FLIP"; }
    const char *name4() const override { return "FLIP"; }

private:

    // Flip
    Vector3f orig_attitude_euler_rad;   // original vehicle attitude before flip

    enum class FlipState : uint8_t {
        Start,
        Roll,
        Pitch_A,
        Pitch_B,
        Recover,
        Abandon
    };
    FlipState _state;                   // current state of flip
    Mode::Number  orig_control_mode;    // flight mode when flip was initiated
    uint32_t start_time_ms;             // time since flip began
    int8_t roll_dir;                    // roll direction (-1 = roll left, 1 = roll right)
    int8_t pitch_dir;                   // pitch direction (-1 = pitch forward, 1 = pitch back)
};


#if MODE_FLOWHOLD_ENABLED
/*
  class to support FLOWHOLD mode, which is a position hold mode using
  optical flow directly, avoiding the need for a rangefinder
 */

class ModeFlowHold : public Mode {
public:
    // need a constructor for parameters
    ModeFlowHold(void);
    Number mode_number() const override { return Number::FLOWHOLD; }

    bool init(bool ignore_checks) override;
    void run(void) override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }
    bool allows_flip() const override { return true; }

    static const struct AP_Param::GroupInfo var_info[];

protected:
    const char *name() const override { return "FLOWHOLD"; }
    const char *name4() const override { return "FHLD"; }

private:

    // FlowHold states
    enum FlowHoldModeState {
        FlowHold_MotorStopped,
        FlowHold_Takeoff,
        FlowHold_Flying,
        FlowHold_Landed
    };

    // calculate attitude from flow data
    void flow_to_angle(Vector2f &bf_angle);

    LowPassFilterConstDtVector2f flow_filter;

    bool flowhold_init(bool ignore_checks);
    void flowhold_run();
    void flowhold_flow_to_angle(Vector2f &angle, bool stick_input);
    void update_height_estimate(void);

    // minimum assumed height
    const float height_min_m = 0.1f;

    // maximum scaling height
    const float height_max = 3.0f;

    AP_Float flow_max;
    AC_PI_2D flow_pi_xy{0.2f, 0.3f, 3000, 5, 0.0025f};
    AP_Float flow_filter_hz;
    AP_Int8  flow_min_quality;
    AP_Int8  brake_rate_dps;

    float quality_filtered;

    uint8_t log_counter;
    bool limited;
    Vector2f xy_I;

    // accumulated INS delta velocity in north-east form since last flow update
    Vector2f delta_velocity_ne_ms;

    // last flow rate in radians/sec in north-east axis
    Vector2f last_flow_rate_rads;

    // timestamp of last flow data
    uint32_t last_flow_ms;

    float last_ins_height_m;
    float height_offset_m;

    // are we braking after pilot input?
    bool braking;

    // last time there was significant stick input
    uint32_t last_stick_input_ms;
};
#endif // MODE_FLOWHOLD_ENABLED


class ModeGuided : public Mode {

public:
#if AP_EXTERNAL_CONTROL_ENABLED
    friend class AP_ExternalControl_Copter;
#endif

    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::GUIDED; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override;
    bool is_autopilot() const override { return true; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool in_guided_mode() const override { return true; }

    bool requires_terrain_failsafe() const override { return true; }

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Return the type of this mode for use by advanced failsafe
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    // Return true if the throttle high arming check can be skipped when arming from GCS or Scripting
    bool allows_GCS_or_SCR_arming_with_throttle_high() const override { return true; }

    // Sets guided's angular target submode: Using a rotation quaternion, angular velocity, and climbrate or thrust (depends on user option)
    // attitude_quat: IF zero: ang_vel (angular velocity) must be provided even if all zeroes
    //                IF non-zero: attitude_control is performed using both the attitude quaternion and angular velocity
    // ang_vel: angular velocity (rad/s)
    // climb_rate_cms_or_thrust: represents either the climb_rate (cm/s) or thrust scaled from [0, 1], unitless
    // use_thrust: IF true: climb_rate_cms_or_thrust represents thrust
    //             IF false: climb_rate_cms_or_thrust represents climb_rate (cm/s)
    void set_angle(const Quaternion &attitude_quat, const Vector3f &ang_vel, float climb_rate_cms_or_thrust, bool use_thrust);

    bool set_destination(const Vector3f& destination, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false, bool terrain_alt = false);
    bool set_destination(const Location& dest_loc, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false);
    bool get_wp(Location &loc) const override;
    void set_accel(const Vector3f& acceleration, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false, bool log_request = true);
    void set_velocity(const Vector3f& velocity, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false, bool log_request = true);
    void set_velaccel(const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false, bool log_request = true);
    bool set_destination_posvel(const Vector3f& destination, const Vector3f& velocity, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false);
    bool set_destination_posvelaccel(const Vector3f& destination, const Vector3f& velocity, const Vector3f& acceleration, bool use_yaw = false, float yaw_rad = 0.0, bool use_yaw_rate = false, float yaw_rate_rads = 0.0, bool yaw_relative = false);

    // get position, velocity and acceleration targets
    const Vector3p& get_target_pos() const;
    const Vector3f& get_target_vel() const;
    const Vector3f& get_target_accel() const;

    // returns true if GUIDED_OPTIONS param suggests SET_ATTITUDE_TARGET's "thrust" field should be interpreted as thrust instead of climb rate
    bool set_attitude_target_provides_thrust() const;
    bool stabilizing_pos_xy() const;
    bool stabilizing_vel_xy() const;
    bool use_wpnav_for_position_control() const;

    void limit_clear();
    void limit_init_time_and_pos();
    void limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    bool limit_check();

    bool is_taking_off() const override;
    
    bool set_speed_xy_cms(float speed_xy_cms) override;
    bool set_speed_up_cms(float speed_up_cms) override;
    bool set_speed_down_cms(float speed_down_cms) override;

    // initialises position controller to implement take-off
    // takeoff_alt_cm is interpreted as alt-above-home (in cm) or alt-above-terrain if a rangefinder is available
    bool do_user_takeoff_start(float takeoff_alt_cm) override;

    enum class SubMode {
        TakeOff,
        WP,
        Pos,
        PosVelAccel,
        VelAccel,
        Accel,
        Angle,
    };

    SubMode submode() const { return guided_mode; }

    void angle_control_start();
    void angle_control_run();

    // return guided mode timeout in milliseconds. Only used for velocity, acceleration, angle control, and angular rate control
    uint32_t get_timeout_ms() const;

    bool use_pilot_yaw() const override;

    // pause continue in guided mode
    bool pause() override;
    bool resume() override;

    // true if weathervaning is allowed in guided
#if WEATHERVANE_ENABLED
    bool allows_weathervaning(void) const override;
#endif

protected:

    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

    float wp_distance_m() const override;
    float wp_bearing_deg() const override;
    float crosstrack_error_m() const override;

private:

    // enum for GUID_OPTIONS parameter
    enum class Option : uint32_t {
        AllowArmingFromTX   = (1U << 0),
        // this bit is still available, pilot yaw was mapped to bit 2 for symmetry with auto
        IgnorePilotYaw      = (1U << 2),
        SetAttitudeTarget_ThrustAsThrust = (1U << 3),
        DoNotStabilizePositionXY = (1U << 4),
        DoNotStabilizeVelocityXY = (1U << 5),
        WPNavUsedForPosControl = (1U << 6),
        AllowWeatherVaning = (1U << 7)
    };

    // returns true if the Guided-mode-option is set (see GUID_OPTIONS)
    bool option_is_enabled(Option option) const;

    // wp controller
    void wp_control_start();
    void wp_control_run();

    void pva_control_start();
    void pos_control_start();
    void accel_control_start();
    void velaccel_control_start();
    void posvelaccel_control_start();
    void takeoff_run();
    void pos_control_run();
    void accel_control_run();
    void velaccel_control_run();
    void pause_control_run();
    void posvelaccel_control_run();
    void set_yaw_state_rad(bool use_yaw, float yaw_rad, bool use_yaw_rate, float yaw_rate_rads, bool relative_angle);

    // controls which controller is run (pos or vel):
    static SubMode guided_mode;
    static bool send_notification;     // used to send one time notification to ground station
    static bool takeoff_complete;      // true once takeoff has completed (used to trigger retracting of landing gear)

    // guided mode is paused or not
    static bool _paused;
};

#if AP_SCRIPTING_ENABLED
// Mode which behaves as guided with custom mode number and name
class ModeGuidedCustom : public ModeGuided {
public:
    // constructor registers custom number and names
    ModeGuidedCustom(const Number _number, const char* _full_name, const char* _short_name);

    bool init(bool ignore_checks) override;

    Number mode_number() const override { return number; }

    const char *name() const override { return full_name; }
    const char *name4() const override { return short_name; }

    // State object which can be edited by scripting
    AP_Vehicle::custom_mode_state state;

private:
    const Number number;
    const char* full_name;
    const char* short_name;
};
#endif

class ModeGuidedNoGPS : public ModeGuided {

public:
    // inherit constructor
    using ModeGuided::Mode;
    Number mode_number() const override { return Number::GUIDED_NOGPS; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "GUIDED_NOGPS"; }
    const char *name4() const override { return "GNGP"; }

private:

};


class ModeLand : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::LAND; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return true; }

    bool is_landing() const override { return true; };

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Return the type of this mode for use by advanced failsafe
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    void do_not_use_GPS();

    // returns true if LAND mode is trying to control X/Y position
    bool controlling_position() const { return control_position; }

    void set_land_pause(bool new_value) { land_pause = new_value; }

protected:

    const char *name() const override { return "LAND"; }
    const char *name4() const override { return "LAND"; }

private:

    void gps_run();
    void nogps_run();

    bool control_position; // true if we are using an external reference to control position

    uint32_t land_start_time;
    bool land_pause;
};


class ModeLoiter : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::LOITER; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool allows_autotune() const override { return true; }
    bool allows_auto_trim() const override { return true; }

#if FRAME_CONFIG == HELI_FRAME
    bool allows_inverted() const override { return true; };
#endif

#if AC_PRECLAND_ENABLED
    void set_precision_loiter_enabled(bool value) { _precision_loiter_enabled = value; }
#endif

protected:

    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

    float wp_distance_m() const override;
    float wp_bearing_deg() const override;
    float crosstrack_error_m() const override { return pos_control->crosstrack_error_m();}

#if AC_PRECLAND_ENABLED
    bool do_precision_loiter();
    void precision_loiter_xy();
#endif

private:

#if AC_PRECLAND_ENABLED
    bool _precision_loiter_enabled;
    bool _precision_loiter_active; // true if user has switched on prec loiter
#endif

};


class ModePosHold : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::POSHOLD; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }
    bool allows_autotune() const override { return true; }
    bool allows_auto_trim() const override { return true;}

protected:

    const char *name() const override { return "POSHOLD"; }
    const char *name4() const override { return "PHLD"; }

private:

    void update_pilot_lean_angle_cd(float &lean_angle_filtered, float &lean_angle_raw);
    float mix_controls(float mix_ratio, float first_control, float second_control);
    void update_brake_angle_from_velocity(float &brake_angle_cd, float velocity_cms);
    void init_wind_comp_estimate();
    void update_wind_comp_estimate();
    void get_wind_comp_lean_angles(float &roll_angle_cd, float &pitch_angle_cd);
    void roll_controller_to_pilot_override();
    void pitch_controller_to_pilot_override();

    enum class RPMode {
        PILOT_OVERRIDE=0,            // pilot is controlling this axis (i.e. roll or pitch)
        BRAKE,                       // this axis is braking towards zero
        BRAKE_READY_TO_LOITER,       // this axis has completed braking and is ready to enter loiter mode (both modes must be this value before moving to next stage)
        BRAKE_TO_LOITER,             // both vehicle's axis (roll and pitch) are transitioning from braking to loiter mode (braking and loiter controls are mixed)
        LOITER,                      // both vehicle axis are holding position
        CONTROLLER_TO_PILOT_OVERRIDE // pilot has input controls on this axis and this axis is transitioning to pilot override (other axis will transition to brake if no pilot input)
    };

    RPMode roll_mode;
    RPMode pitch_mode;

    // pilot input related variables
    float pilot_roll_cd;  // filtered roll lean angle commanded by the pilot. Slowly returns to zero when stick is released
    float pilot_pitch_cd; // filtered pitch lean angle commanded by the pilot. Slowly returns to zero when stick is released


    // braking related variables
    struct {
        bool  time_updated_roll;            // true if braking timeout on roll axis has been re-estimated
        bool  time_updated_pitch;           // true if braking timeout on pitch axis has been re-estimated
        float gain;                         // braking gain used to convert velocity to lean angle
        float roll_cd;                      // braking roll angle in centidegrees
        float pitch_cd;                     // braking pitch angle in centidegrees
        uint32_t start_time_roll_ms;        // time (ms) when braking on roll axis begins
        uint32_t start_time_pitch_ms;       // time (ms) when braking on pitch axis begins
        float angle_max_roll_cd;            // peak roll angle (deg x100) during braking, used to detect vehicle flattening
        float angle_max_pitch_cd;           // peak pitch angle (deg x100) during braking, used to detect vehicle flattening
        uint32_t loiter_transition_start_time_ms;   // time (ms) when transition from brake to loiter started
    } brake;


    // loiter related variables
    uint32_t controller_to_pilot_start_time_roll_ms;   // time (ms) when transition from controller to pilot roll input began
    uint32_t controller_to_pilot_start_time_pitch_ms;  // time (ms) when transition from controller to pilot pitch input began

    float controller_final_roll_cd;   // final roll output (deg x100) from controller before transition to pilot input
    float controller_final_pitch_cd;  // final pitch output (deg x100) from controller before transition to pilot input

    // wind compensation related variables
    Vector2f wind_comp_ef;              // wind compensation acceleration vector (earth frame), low-pass filtered
    float wind_comp_roll_cd;            // roll angle (deg x100) to counter wind based on earth-frame lean
    float wind_comp_pitch_cd;           // pitch angle (deg x100) to counter wind based on earth-frame lean
    uint32_t wind_comp_start_time_ms;   // time (ms) when wind compensation updates are started

    // final output
    float roll_cd;   // final roll angle sent to attitude controller
    float pitch_cd;  // final pitch angle sent to attitude controller
};


class ModeRTL : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::RTL; }

    bool init(bool ignore_checks) override;
    void run() override {
        return run(true);
    }
    void run(bool disarm_on_land);

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return true; }

    bool requires_terrain_failsafe() const override { return true; }

#if AP_COPTER_ADVANCED_FAILSAFE_ENABLED
    // Return the type of this mode for use by advanced failsafe
    AP_AdvancedFailsafe_Copter::control_mode afs_mode() const override { return AP_AdvancedFailsafe_Copter::control_mode::AFS_AUTO; }
#endif

    // for reporting to GCS
    bool get_wp(Location &loc) const override;

    bool use_pilot_yaw() const override;

    bool set_speed_xy_cms(float speed_xy_cms) override;
    bool set_speed_up_cms(float speed_up_cms) override;
    bool set_speed_down_cms(float speed_down_cms) override;

    // RTL states
    enum class SubMode : uint8_t {
        STARTING,
        INITIAL_CLIMB,
        RETURN_HOME,
        LOITER_AT_HOME,
        FINAL_DESCENT,
        LAND
    };
    SubMode state() { return _state; }

    // this should probably not be exposed
    bool state_complete() const { return _state_complete; }

    virtual bool is_landing() const override;

    void restart_without_terrain();

    // enum for RTL_ALT_TYPE parameter
    enum class RTLAltType : int8_t {
        RELATIVE = 0,
        TERRAIN = 1
    };
    ModeRTL::RTLAltType get_alt_type() const;

protected:

    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

    // for reporting to GCS
    float wp_distance_m() const override;
    float wp_bearing_deg() const override;
    float crosstrack_error_m() const override { return wp_nav->crosstrack_error_m();}

    void descent_start();
    void descent_run();
    void land_start();
    void land_run(bool disarm_on_land);

    void set_descent_target_alt(uint32_t alt) { rtl_path.descent_target.alt = alt; }

private:

    void climb_start();
    void return_start();
    void climb_return_run();
    void loiterathome_start();
    void loiterathome_run();
    void build_path();
    void compute_return_target();

    SubMode _state = SubMode::INITIAL_CLIMB;  // records state of rtl (initial climb, returning home, etc)
    bool _state_complete = false; // set to true if the current state is completed

    struct {
        // NEU w/ Z element alt-above-ekf-origin unless use_terrain is true in which case Z element is alt-above-terrain
        Location origin_point;
        Location climb_target;
        Location return_target;
        Location descent_target;
        bool land;
    } rtl_path;

    // return target alt type
    enum class ReturnTargetAltType {
        RELATIVE = 0,
        RANGEFINDER = 1,
        TERRAINDATABASE = 2
    };

    // Loiter timer - Records how long we have been in loiter
    uint32_t _loiter_start_time;

    bool terrain_following_allowed;

    // enum for RTL_OPTIONS parameter
    enum class Options : int32_t {
        // First pair of bits are still available, pilot yaw was mapped to bit 2 for symmetry with auto
        IgnorePilotYaw    = (1U << 2),
    };

};


class ModeSmartRTL : public ModeRTL {

public:
    // inherit constructor
    using ModeRTL::Mode;
    Number mode_number() const override { return Number::SMART_RTL; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    bool is_autopilot() const override { return true; }

    void save_position();
    void exit() override;

    bool is_landing() const override;
    bool use_pilot_yaw() const override;

    // Safe RTL states
    enum class SubMode : uint8_t {
        WAIT_FOR_PATH_CLEANUP,
        PATH_FOLLOW,
        PRELAND_POSITION,
        DESCEND,
        LAND
    };

protected:

    const char *name() const override { return "SMARTRTL"; }
    const char *name4() const override { return "SRTL"; }

    // for reporting to GCS
    bool get_wp(Location &loc) const override;
    float wp_distance_m() const override;
    float wp_bearing_deg() const override;
    float crosstrack_error_m() const override { return wp_nav->crosstrack_error_m();}

private:

    void wait_cleanup_run();
    void path_follow_run();
    void pre_land_position_run();
    void land();
    SubMode smart_rtl_state = SubMode::PATH_FOLLOW;

    // keep track of how long we have failed to get another return
    // point while following our path home.  If we take too long we
    // may choose to land the vehicle.
    uint32_t path_follow_last_pop_fail_ms;

    // backup last popped point so that it can be restored to the path
    // if vehicle exits SmartRTL mode before reaching home. invalid if zero
    Vector3f dest_NED_backup;
};


class ModeSport : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::SPORT; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }

protected:

    const char *name() const override { return "SPORT"; }
    const char *name4() const override { return "SPRT"; }

private:

};


class ModeStabilize : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::STABILIZE; }

    virtual void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool allows_save_trim() const override { return true; }
    bool allows_auto_trim() const override { return true; }
    bool allows_autotune() const override { return true; }
    bool allows_flip() const override { return true; }
    bool allows_entry_in_rc_failsafe() const override { return false; }

protected:

    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

private:

};

#if FRAME_CONFIG == HELI_FRAME
class ModeStabilize_Heli : public ModeStabilize {

public:
    // inherit constructor
    using ModeStabilize::Mode;

    bool init(bool ignore_checks) override;
    void run() override;

    bool allows_inverted() const override { return true; };

protected:

private:

};
#endif

class ModeSystemId : public Mode {

public:
    ModeSystemId(void);
    Number mode_number() const override { return Number::SYSTEMID; }

    bool init(bool ignore_checks) override;
    void run() override;
    void exit() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };
    bool is_autopilot() const override { return false; }
    bool logs_attitude() const override { return true; }

    void set_magnitude(float input) { waveform_magnitude.set(input); }

    static const struct AP_Param::GroupInfo var_info[];

    Chirp chirp_input;

protected:

    const char *name() const override { return "SYSTEMID"; }
    const char *name4() const override { return "SYSI"; }

private:

    void log_data() const;
    bool is_poscontrol_axis_type() const;

    enum class AxisType {
        NONE = 0,               // none
        INPUT_ROLL = 1,         // angle input roll axis is being excited
        INPUT_PITCH = 2,        // angle pitch axis is being excited
        INPUT_YAW = 3,          // angle yaw axis is being excited
        RECOVER_ROLL = 4,       // angle roll axis is being excited
        RECOVER_PITCH = 5,      // angle pitch axis is being excited
        RECOVER_YAW = 6,        // angle yaw axis is being excited
        RATE_ROLL = 7,          // rate roll axis is being excited
        RATE_PITCH = 8,         // rate pitch axis is being excited
        RATE_YAW = 9,           // rate yaw axis is being excited
        MIX_ROLL = 10,          // mixer roll axis is being excited
        MIX_PITCH = 11,         // mixer pitch axis is being excited
        MIX_YAW = 12,           // mixer pitch axis is being excited
        MIX_THROTTLE = 13,      // mixer throttle axis is being excited
        DISTURB_POS_LAT = 14,   // lateral body axis measured position is being excited
        DISTURB_POS_LONG = 15,  // longitudinal body axis measured position is being excited
        DISTURB_VEL_LAT = 16,   // lateral body axis measured velocity is being excited
        DISTURB_VEL_LONG = 17,  // longitudinal body axis measured velocity is being excited
        INPUT_VEL_LAT = 18,     // lateral body axis commanded velocity is being excited
        INPUT_VEL_LONG = 19,    // longitudinal body axis commanded velocity is being excited
    };

    AP_Int8 axis;               // Controls which axis are being excited. Set to non-zero to display other parameters
    AP_Float waveform_magnitude;// Magnitude of chirp waveform
    AP_Float frequency_start;   // Frequency at the start of the chirp
    AP_Float frequency_stop;    // Frequency at the end of the chirp
    AP_Float time_fade_in;      // Time to reach maximum amplitude of chirp
    AP_Float time_record;       // Time taken to complete the chirp waveform
    AP_Float time_fade_out;     // Time to reach zero amplitude after chirp finishes

    bool att_bf_feedforward;    // Setting of attitude_control->get_bf_feedforward
    float waveform_time;        // Time reference for waveform
    float waveform_sample;      // Current waveform sample
    float waveform_freq_rads;   // Instantaneous waveform frequency
    float time_const_freq;      // Time at constant frequency before chirp starts
    int8_t log_subsample;       // Subsample multiple for logging.
    Vector2f target_vel;        // target velocity for position controller modes
    Vector2f target_pos;       // target position
    Vector2f input_vel_last;    // last cycle input velocity
    // System ID states
    enum class SystemIDModeState {
        SYSTEMID_STATE_STOPPED,
        SYSTEMID_STATE_TESTING
    } systemid_state;
};

class ModeThrow : public Mode {

public:
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::THROW; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }

    // Throw types
    enum class ThrowType {
        Upward = 0,
        Drop = 1
    };

    enum class PreThrowMotorState {
        STOPPED = 0,
        RUNNING = 1,
    };

protected:

    const char *name() const override { return "THROW"; }
    const char *name4() const override { return "THRW"; }

private:

    bool throw_detected();
    bool throw_position_good() const;
    bool throw_height_good() const;
    bool throw_attitude_good() const;

    // Throw stages
    enum ThrowModeStage {
        Throw_Disarmed,
        Throw_Detecting,
        Throw_Wait_Throttle_Unlimited,
        Throw_Uprighting,
        Throw_HgtStabilise,
        Throw_PosHold
    };

    ThrowModeStage stage = Throw_Disarmed;
    ThrowModeStage prev_stage = Throw_Disarmed;
    uint32_t last_log_ms;
    bool nextmode_attempted;
    uint32_t free_fall_start_ms;    // system time free fall was detected
    float free_fall_start_velz;     // vertical velocity when free fall was detected
};

#if MODE_TURTLE_ENABLED
class ModeTurtle : public Mode {

public:
    // inherit constructors
    using Mode::Mode;
    Number mode_number() const override { return Number::TURTLE; }

    bool init(bool ignore_checks) override;
    void run() override;
    void exit() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(AP_Arming::Method method) const override;
    bool is_autopilot() const override { return false; }
    void change_motor_direction(bool reverse);
    void output_to_motors() override;
    bool allows_entry_in_rc_failsafe() const override { return false; }

protected:
    const char *name() const override { return "TURTLE"; }
    const char *name4() const override { return "TRTL"; }

private:
    void arm_motors();
    void disarm_motors();

    float motors_output;
    Vector2f motors_input;
    uint32_t last_throttle_warning_output_ms;

    // Semaphore to protect the motors from the arming state
    HAL_Semaphore msem;
    bool shutdown;
};
#endif

// modes below rely on Guided mode so must be declared at the end (instead of in alphabetical order)

#if AP_ADSB_AVOIDANCE_ENABLED
class ModeAvoidADSB : public ModeGuided {

public:
    // inherit constructor
    using ModeGuided::Mode;
    Number mode_number() const override { return Number::AVOID_ADSB; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    bool is_autopilot() const override { return true; }

    bool set_velocity(const Vector3f& velocity_neu);

protected:

    const char *name() const override { return "AVOID_ADSB"; }
    const char *name4() const override { return "AVOI"; }

private:

};
#endif  // AP_ADSB_AVOIDANCE_ENABLED

#if MODE_FOLLOW_ENABLED
class ModeFollow : public ModeGuided {

public:

    // inherit constructor
    using ModeGuided::Mode;
    Number mode_number() const override { return Number::FOLLOW; }

    bool init(bool ignore_checks) override;
    void exit() override;
    void run() override;

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; }
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "FOLLOW"; }
    const char *name4() const override { return "FOLL"; }

    // for reporting to GCS
    bool get_wp(Location &loc) const override;
    float  wp_distance_m() const override;
    float wp_bearing_deg() const override;

    uint32_t last_log_ms;   // system time of last time desired velocity was logging
};
#endif

class ModeZigZag : public Mode {        

public:
    ModeZigZag(void);

    // Inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::ZIGZAG; }

    enum class Destination : uint8_t {
        A,  // Destination A
        B,  // Destination B
    };

    enum class Direction : uint8_t {
        FORWARD,        // moving forward from the yaw direction
        RIGHT,          // moving right from the yaw direction
        BACKWARD,       // moving backward from the yaw direction
        LEFT,           // moving left from the yaw direction
    } zigzag_direction;

    bool init(bool ignore_checks) override;
    void exit() override;
    void run() override;

    // auto control methods.  copter flies grid pattern
    void run_auto();
    void suspend_auto();
    void init_auto();

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; }
    bool is_autopilot() const override { return true; }
    bool has_user_takeoff(bool must_navigate) const override { return true; }

    // save current position as A or B.  If both A and B have been saved move to the one specified
    void save_or_move_to_destination(Destination ab_dest);

    // return manual control to the pilot
    void return_to_manual_control(bool maintain_target);

    static const struct AP_Param::GroupInfo var_info[];

protected:

    const char *name() const override { return "ZIGZAG"; }
    const char *name4() const override { return "ZIGZ"; }
    float wp_distance_m() const override;
    float wp_bearing_deg() const override;
    float crosstrack_error_m() const override;

private:

    void auto_control();
    void manual_control();
    bool reached_destination();
    bool calculate_next_dest(Destination ab_dest, bool use_wpnav_alt, Vector3f& next_dest, bool& terrain_alt) const;
    void spray(bool b);
    bool calculate_side_dest(Vector3f& next_dest, bool& terrain_alt) const;
    void move_to_side();

    Vector2f dest_A_ne_cm;    // in NEU frame in cm relative to ekf origin
    Vector2f dest_B_ne_cm;    // in NEU frame in cm relative to ekf origin
    Vector3f current_dest; // current target destination (use for resume after suspending)
    bool current_terr_alt;

    // parameters
    AP_Int8  _auto_enabled;    // top level enable/disable control
#if HAL_SPRAYER_ENABLED
    AP_Int8  _spray_enabled;   // auto spray enable/disable
#endif
    AP_Int8  _wp_delay_s;      // delay for zigzag waypoint
    AP_Float _side_dist_m;     // sideways distance
    AP_Int8  _direction;       // sideways direction
    AP_Int16 _line_num;        // total number of lines

    enum ZigZagState {
        STORING_POINTS, // storing points A and B, pilot has manual control
        AUTO,           // after A and B defined, pilot toggle the switch from one side to the other, vehicle flies autonomously
        MANUAL_REGAIN   // pilot toggle the switch to middle position, has manual control
    } stage;

    enum AutoState {
        MANUAL,         // not in ZigZag Auto
        AB_MOVING,      // moving from A to B or from B to A
        SIDEWAYS,       // moving to sideways
    } auto_stage;

    uint32_t reach_wp_time_ms = 0;  // time since vehicle reached destination (or zero if not yet reached)
    Destination ab_dest_stored;     // store the current destination
    bool is_auto;                   // enable zigzag auto feature which is automate both AB and sideways
    uint16_t line_count = 0;        // current line number
    int16_t line_num = 0;           // target line number
    bool is_suspended;              // true if zigzag auto is suspended
};

#if MODE_AUTOROTATE_ENABLED
class ModeAutorotate : public Mode {

public:

    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::AUTOROTATE; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool is_autopilot() const override { return true; }
    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return false; };

    static const struct AP_Param::GroupInfo  var_info[];

protected:

    const char *name() const override { return "AUTOROTATE"; }
    const char *name4() const override { return "AROT"; }

private:

    uint32_t _entry_time_start_ms;  // time remaining until entry phase moves on to glide phase
    uint32_t _last_logged_ms;       // used for timing slow rate autorotation log

    enum class Phase {
        ENTRY_INIT,
        ENTRY,
        GLIDE_INIT,
        GLIDE,
        FLARE_INIT,
        FLARE,
        TOUCH_DOWN_INIT,
        TOUCH_DOWN,
        LANDED_INIT,
        LANDED,
    } current_phase;

};
#endif
