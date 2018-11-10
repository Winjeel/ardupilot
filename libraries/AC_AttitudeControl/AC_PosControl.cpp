#include <AP_HAL/AP_HAL.h>
#include "AC_PosControl.h"
#include <AP_Math/AP_Math.h>
#include <DataFlash/DataFlash.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 // default gains for Plane
 # define POSCONTROL_POS_Z_P                    1.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_ACC_Z_P                    0.3f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    1.0f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              10.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.02f   // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   1.4f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   0.0f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.0f   // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
 // default gains for Sub
 # define POSCONTROL_POS_Z_P                    3.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    8.0f    // vertical velocity controller P gain default
 # define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    0.1f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 100     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   1.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   0.5f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.0f    // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#else
 // default gains for Copter / TradHeli
 # define POSCONTROL_POS_Z_P                    1.0f    // vertical position controller P gain default
 # define POSCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default
 # define POSCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 # define POSCONTROL_ACC_Z_I                    1.0f    // vertical acceleration controller I gain default
 # define POSCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define POSCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
 # define POSCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define POSCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define POSCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define POSCONTROL_VEL_XY_P                   2.0f    // horizontal velocity controller P gain default
 # define POSCONTROL_VEL_XY_I                   1.0f    // horizontal velocity controller I gain default
 # define POSCONTROL_VEL_XY_D                   0.5f    // horizontal velocity controller D gain default
 # define POSCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define POSCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define POSCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#endif

const AP_Param::GroupInfo AC_PosControl::var_info[] = {
    // @Param: _WING_Z
    // @DisplayName: Fraction of wing normal force to compensate for in throttle calculation
    // @Description: The fraction of measured wing normal acceleration the accel to throttle calculation will account for.
    // @Range: 0.0 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("_WING_Z", 0, AC_PosControl, _accel_z_wing_k, 0.7f),

    // @Param: _ACC_XY_FILT
    // @DisplayName: XY Acceleration filter cutoff frequency
    // @Description: Lower values will slow the response of the navigation controller and reduce twitchiness
    // @Units: Hz
    // @Range: 0.5 5
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_ACC_XY_FILT", 1, AC_PosControl, _accel_xy_filt_hz, POSCONTROL_ACCEL_FILTER_HZ),

    // @Param: _POSZ_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_z, "_POSZ_", 2, AC_PosControl, AC_P),

    // @Param: _VELZ_P
    // @DisplayName: Velocity (vertical) controller P gain
    // @Description: Velocity (vertical) controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_vel_z, "_VELZ_", 3, AC_PosControl, AC_P),

    // @Param: _ACCZ_P
    // @DisplayName: Acceleration (vertical) controller P gain
    // @Description: Acceleration (vertical) controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @Increment: 0.05
    // @User: Standard

    // @Param: _ACCZ_I
    // @DisplayName: Acceleration (vertical) controller I gain
    // @Description: Acceleration (vertical) controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: _ACCZ_IMAX
    // @DisplayName: Acceleration (vertical) controller I gain maximum
    // @Description: Acceleration (vertical) controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: d%
    // @User: Standard

    // @Param: _ACCZ_D
    // @DisplayName: Acceleration (vertical) controller D gain
    // @Description: Acceleration (vertical) controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _ACCZ_FILT
    // @DisplayName: Acceleration (vertical) controller filter
    // @Description: Filter applied to acceleration to reduce noise.  Lower values reduce noise but add delay.
    // @Range: 1.000 100.000
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_accel_z, "_ACCZ_", 4, AC_PosControl, AC_PID),

    // @Param: _POSXY_P
    // @DisplayName: Position (horizonal) controller P gain
    // @Description: Position controller P gain.  Converts the distance (in the latitude direction) to the target location into a desired speed which is then passed to the loiter latitude rate controller
    // @Range: 0.500 2.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_xy, "_POSXY_", 5, AC_PosControl, AC_P),

    // @Param: _VELXY_P
    // @DisplayName: Velocity (horizontal) P gain
    // @Description: Velocity (horizontal) P gain.  Converts the difference between desired velocity to a target acceleration
    // @Range: 0.1 6.0
    // @Increment: 0.1
    // @User: Advanced

    // @Param: _VELXY_I
    // @DisplayName: Velocity (horizontal) I gain
    // @Description: Velocity (horizontal) I gain.  Corrects long-term difference in desired velocity to a target acceleration
    // @Range: 0.02 1.00
    // @Increment: 0.01
    // @User: Advanced

    // @Param: _VELXY_D
    // @DisplayName: Velocity (horizontal) D gain
    // @Description: Velocity (horizontal) D gain.  Corrects short-term changes in velocity
    // @Range: 0.00 1.00
    // @Increment: 0.001
    // @User: Advanced

    // @Param: _VELXY_IMAX
    // @DisplayName: Velocity (horizontal) integrator maximum
    // @Description: Velocity (horizontal) integrator maximum.  Constrains the target acceleration that the I gain will output
    // @Range: 0 4500
    // @Increment: 10
    // @Units: cm/s/s
    // @User: Advanced

    // @Param: _VELXY_FILT
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced

    // @Param: _VELXY_D_FILT
    // @DisplayName: Velocity (horizontal) input filter
    // @Description: Velocity (horizontal) input filter.  This filter (in hz) is applied to the input for P and I terms
    // @Range: 0 100
    // @Units: Hz
    // @User: Advanced
    AP_SUBGROUPINFO(_pid_vel_xy, "_VELXY_", 6, AC_PosControl, AC_PID_2D),

    // 7 unused. Used previously by _ANGLE_MAX which was deprecated.

    // 8 unused. Used previously by _WING_XY which was deprecated.

    // 8 unused. Used previously by _TRIM_SPD which was deprecated.

    // 10 unused. Used previously by _TRIM_EXP which was deprecated.

    // 11 unused. Used previously by _TRIM_METHOD which was deprecated.

    // 12 unused. Used previously by _TRIM_BIAS which was deprecated.

    // @Param: _TRIM_TAU
    // @DisplayName: Time constant applied to trim correction
    // @Range: 0.1 5.0
    // @Units: sec
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("_TRIM_TAU", 13, AC_PosControl, _trim_tau, 1.0f),

    // @Param: _AIRSPD_I
    // @DisplayName: Gain from integral of ground velocity error to demanded airspeed.
    // @Range: 0.0 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("_AIRSPD_I", 14, AC_PosControl, _vel_err_i_gain, 1.0f),

    // @Param: _FWD_SPD_MAX
    // @DisplayName: Maximum forward airspeed in VTOL mode.
    // @Units: m/s
    // @Range: 10 20
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("_FWD_SPD_MAX",  15, AC_PosControl, _fwd_spd_max, 15.0f),

    // 16 unused. Used previously by _AFT_SPD_MAX which was deprecated.

    // @Param: _FWD_AZ_GF
    // @DisplayName: Vertical PD Gain Reduction Factor.
    // @Description: Used to compensate for the increased sensitivity from rotor tilt angle to vertical g during high speed VTOL operation due to wing lift.
    // @Range: 0.0 0.9
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("_FWD_AZ_GF",  17, AC_PosControl, _fwd_az_gf, 0.5f),

    // @Param: _FWD_BCOEF
    // @DisplayName: Profile drag ballistic coefficient for forward flight.
    // @Description: Is equivalent to mass / (area * drag_coef). Set initially to a value consistent with profile drag at _FWD_SPD_MAX.
    // @Range: 10.0 1000.0
    // @Increment: 5
    // @User: Advanced
    AP_GROUPINFO("_FWD_BCOEF",  18, AC_PosControl, _fwd_bcoef, 110.0f),

    // @Param: _FWD_INFLOW
    // @DisplayName: Propeller inflow correction factor.
    // @Description: Gain from estimated inflow to demanded throttle and normalised using hover throttle. Used to correct for reduction in thrust when flying forward. For example a value of 0.01 will increase the throttle by 0.1 (10%) when flying with props tilted forward into a 10m/s airflow.
    // @Range: 0.0 0.05
    // @User: Advanced
    AP_GROUPINFO("_FWD_INFLOW",  19, AC_PosControl, _fwd_inflow_thrust_factor, 0.007f),

    // @Param: _LAND_GYRY
    // @DisplayName: Landed detector wing pitch rate threshold.
    // @Description: Wing pitch rate required to trigger landed detector.
    // @Range: 0.5 5.0
    // @Units: rad/sec
    // @User: Advanced
    AP_GROUPINFO("_LAND_GYRY",  20, AC_PosControl, _landed_gyro_y, 2.0f),

    // @Param: _LAND_PITCH
    // @DisplayName: Landed detector wing pitch threshold.
    // @Description: Wing pitch angle required to trigger landed detector.
    // @Range: 0.0 0.05
    // @Units: rad
    // @User: Advanced
    AP_GROUPINFO("_LAND_PITCH",  21, AC_PosControl, _landed_pitch, 0.25f),

    // @Param: _LAND_ACCEL
    // @DisplayName: Landed detector accceleration threshold.
    // @Description: IMU acceleration required to trigger landed detector.
    // @Range: 15.0 30.0
    // @Units: m/s/s
    // @User: Advanced
    AP_GROUPINFO("_LAND_ACCEL",  22, AC_PosControl, _landed_accel, 20.0f),

    // @Param: _FWD_DB_THR
    // @DisplayName: Rotor Drag Brake Threshold.
    // @Description: Decel due to aero drag has to fall below this value before rotors will tilt rearwards to perform a braking manoeuvre. Smaller values make the controller wait for longer before rotors are tilted back past vertical to complete a braking or reversal manoevre which prevents loss of control due to reverse flow through rotors. Increase value if time spent with rotors level during braking manoeuvres is excessive.
    // @Range: 0.05 0.5
    // @Units: g
    // @User: Advanced
    AP_GROUPINFO("_FWD_DB_THR",  23, AC_PosControl, _drag_brake_g_thrhld, 0.15f),


    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PosControl::AC_PosControl(const AP_AHRS_View& ahrs, const AP_AHRS& ahrs_wing, const AP_InertialNav& inav,
                             const AP_Motors& motors, AC_AttitudeControl& attitude_control) :
    _ahrs(ahrs),
    _ahrs_wing(ahrs_wing),
    _inav(inav),
    _motors(motors),
    _attitude_control(attitude_control),
    _p_pos_z(POSCONTROL_POS_Z_P),
    _p_vel_z(POSCONTROL_VEL_Z_P),
    _pid_accel_z(POSCONTROL_ACC_Z_P, POSCONTROL_ACC_Z_I, POSCONTROL_ACC_Z_D, POSCONTROL_ACC_Z_IMAX, POSCONTROL_ACC_Z_FILT_HZ, POSCONTROL_ACC_Z_DT),
    _p_pos_xy(POSCONTROL_POS_XY_P),
    _pid_vel_xy(POSCONTROL_VEL_XY_P, POSCONTROL_VEL_XY_I, POSCONTROL_VEL_XY_D, POSCONTROL_VEL_XY_IMAX, POSCONTROL_VEL_XY_FILT_HZ, POSCONTROL_VEL_XY_FILT_D_HZ, POSCONTROL_DT_50HZ),
    _dt(POSCONTROL_DT_400HZ),
    _speed_down_cms(POSCONTROL_SPEED_DOWN),
    _speed_up_cms(POSCONTROL_SPEED_UP),
    _speed_cms(POSCONTROL_SPEED),
    _accel_z_cms(POSCONTROL_ACCEL_Z),
    _accel_cms(POSCONTROL_ACCEL_XY),
    _leash(POSCONTROL_LEASH_LENGTH_MIN),
    _leash_down_z(POSCONTROL_LEASH_LENGTH_MIN),
    _leash_up_z(POSCONTROL_LEASH_LENGTH_MIN),
    _accel_target_filter(POSCONTROL_ACCEL_FILTER_HZ),
    _roll_target_cd(0.0f),
    _pitch_target_cd(0.0f),
    _vel_xy_integ_length_prev(0.0f),
    _distance_to_target(0.0f),
    _pitch_trim_rad(0.0f),
    _accel_target_xy_updated(false),
    _vel_forward_filt(0.0f),
    _last_log_time_ms(0),
    _vel_err_i_gain_scale(1.0f),
    _taking_off(true),
    _land_expect_time_ms(0),
    _land_detect_time_ms(0),
    _land_pitch_rate_time_ms(0),
    _land_pitch_ang_time_ms(0),
    _land_accz_time_ms(0),
    _is_landed(false)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise flags
    _flags.recalc_leash_z = true;
    _flags.recalc_leash_xy = true;
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_accel_to_lean_xy = true;
    _flags.reset_rate_to_accel_z = true;
    _flags.reset_accel_to_throttle = true;
    _flags.freeze_ff_z = true;
    _flags.use_desvel_ff_z = true;
    _limit.pos_up = true;
    _limit.pos_down = true;
    _limit.vel_up = true;
    _limit.vel_down = true;
    _limit.accel_xy = true;
}

///
/// z-axis position controller
///


/// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
void AC_PosControl::set_dt(float delta_sec)
{
    _dt = delta_sec;

    // update PID controller dt
    _pid_accel_z.set_dt(_dt);
    _pid_vel_xy.set_dt(_dt);

    // update rate z-axis velocity error and wing normal force accel filters
    _vel_error_filter.set_cutoff_frequency(POSCONTROL_VEL_ERROR_CUTOFF_FREQ);
    _wing_lift_accel_filter.set_cutoff_frequency(POSCONTROL_ACCEL_FILTER_HZ);
    _wing_drag_accel_filter.set_cutoff_frequency(POSCONTROL_ACCEL_FILTER_HZ);
}

/// set_max_speed_z - set the maximum climb and descent rates
/// To-Do: call this in the main code as part of flight mode initialisation
void AC_PosControl::set_max_speed_z(float speed_down, float speed_up)
{
    // ensure speed_down is always negative
    speed_down = -fabsf(speed_down);

    if ((fabsf(_speed_down_cms-speed_down) > 1.0f) || (fabsf(_speed_up_cms-speed_up) > 1.0f)) {
        _speed_down_cms = speed_down;
        _speed_up_cms = speed_up;
        _flags.recalc_leash_z = true;
        calc_leash_length_z();
    }
}

/// set_max_accel_z - set the maximum vertical acceleration in cm/s/s
void AC_PosControl::set_max_accel_z(float accel_cmss)
{
    if (fabsf(_accel_z_cms-accel_cmss) > 1.0f) {
        _accel_z_cms = accel_cmss;
        _flags.recalc_leash_z = true;
        calc_leash_length_z();
    }
}

/// set_alt_target_with_slew - adjusts target towards a final altitude target
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
void AC_PosControl::set_alt_target_with_slew(float alt_cm, float dt)
{
    float alt_change = alt_cm-_pos_target.z;

    // do not use z-axis desired velocity feed forward
    _flags.use_desvel_ff_z = false;

    // adjust desired alt if motors have not hit their limits
    if ((alt_change<0 && !_motors.limit.throttle_lower) || (alt_change>0 && !_motors.limit.throttle_upper)) {
        if (!is_zero(dt)) {
            float climb_rate_cms = constrain_float(alt_change/dt, _speed_down_cms, _speed_up_cms);
            _pos_target.z += climb_rate_cms*dt;
            _vel_desired.z = climb_rate_cms;    // recorded for reporting purposes
        }
    }

    // do not let target get too far from current altitude
    float curr_alt = _inav.get_altitude();
    _pos_target.z = constrain_float(_pos_target.z,curr_alt-_leash_down_z,curr_alt+_leash_up_z);
}


/// set_alt_target_from_climb_rate - adjusts target up or down using a climb rate in cm/s
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
void AC_PosControl::set_alt_target_from_climb_rate(float climb_rate_cms, float dt, bool force_descend)
{
    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_down?
    if ((climb_rate_cms<0 && (!_motors.limit.throttle_lower || force_descend)) || (climb_rate_cms>0 && !_motors.limit.throttle_upper && !_limit.pos_up)) {
        _pos_target.z += climb_rate_cms * dt;
    }

    // do not use z-axis desired velocity feed forward
    // vel_desired set to desired climb rate for reporting and land-detector
    _flags.use_desvel_ff_z = false;
    _vel_desired.z = climb_rate_cms;
}

/// set_alt_target_from_climb_rate_ff - adjusts target up or down using a climb rate in cm/s using feed-forward
///     should be called continuously (with dt set to be the expected time between calls)
///     actual position target will be moved no faster than the speed_down and speed_up
///     target will also be stopped if the motors hit their limits or leash length is exceeded
///     set force_descend to true during landing to allow target to move low enough to slow the motors
void AC_PosControl::set_alt_target_from_climb_rate_ff(float climb_rate_cms, float dt, bool force_descend)
{
    // calculated increased maximum acceleration if over speed
    float accel_z_cms = _accel_z_cms;
    if (_vel_desired.z < _speed_down_cms && !is_zero(_speed_down_cms)) {
        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_down_cms;
    }
    if (_vel_desired.z > _speed_up_cms && !is_zero(_speed_up_cms)) {
        accel_z_cms *= POSCONTROL_OVERSPEED_GAIN_Z * _vel_desired.z / _speed_up_cms;
    }
    accel_z_cms = constrain_float(accel_z_cms, 0.0f, 750.0f);

    // jerk_z is calculated to reach full acceleration in 1000ms.
    float jerk_z = accel_z_cms * POSCONTROL_JERK_RATIO;

    float accel_z_max = MIN(accel_z_cms, safe_sqrt(2.0f*fabsf(_vel_desired.z - climb_rate_cms)*jerk_z));

    _accel_last_z_cms += jerk_z * dt;
    _accel_last_z_cms = MIN(accel_z_max, _accel_last_z_cms);

    float vel_change_limit = _accel_last_z_cms * dt;
    _vel_desired.z = constrain_float(climb_rate_cms, _vel_desired.z-vel_change_limit, _vel_desired.z+vel_change_limit);
    _flags.use_desvel_ff_z = true;

    // adjust desired alt if motors have not hit their limits
    // To-Do: add check of _limit.pos_down?
    if ((_vel_desired.z<0 && (!_motors.limit.throttle_lower || force_descend)) || (_vel_desired.z>0 && !_motors.limit.throttle_upper && !_limit.pos_up)) {
        _pos_target.z += _vel_desired.z * dt;
    }
}

/// add_takeoff_climb_rate - adjusts alt target up or down using a climb rate in cm/s
///     should be called continuously (with dt set to be the expected time between calls)
///     almost no checks are performed on the input
void AC_PosControl::add_takeoff_climb_rate(float climb_rate_cms, float dt)
{
    _pos_target.z += climb_rate_cms * dt;
}

/// shift altitude target (positive means move altitude up)
void AC_PosControl::shift_alt_target(float z_cm)
{
    _pos_target.z += z_cm;

    // freeze feedforward to avoid jump
    if (!is_zero(z_cm)) {
        freeze_ff_z();
    }
}

/// relax_alt_hold_controllers - set all desired and targets to measured
void AC_PosControl::relax_alt_hold_controllers(float throttle_setting)
{
    _pos_target.z = _inav.get_altitude();
    _vel_desired.z = 0.0f;
    _flags.use_desvel_ff_z = false;
    _vel_target.z = _inav.get_velocity_z();
    _vel_last.z = _inav.get_velocity_z();
    _accel_desired.z = 0.0f;
    _accel_last_z_cms = 0.0f;
    _accel_target.z = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;
    _flags.reset_accel_to_throttle = true;
    _pid_accel_z.set_integrator((throttle_setting-_motors.get_throttle_hover())*1000.0f);
}

// get_alt_error - returns altitude error in cm
float AC_PosControl::get_alt_error() const
{
    return (_pos_target.z - _inav.get_altitude());
}

/// set_target_to_stopping_point_z - returns reasonable stopping altitude in cm above home
void AC_PosControl::set_target_to_stopping_point_z()
{
    // check if z leash needs to be recalculated
    calc_leash_length_z();

    get_stopping_point_z(_pos_target);
}

/// get_stopping_point_z - calculates stopping point based on current position, velocity, vehicle acceleration
void AC_PosControl::get_stopping_point_z(Vector3f& stopping_point) const
{
    const float curr_pos_z = _inav.get_altitude();
    float curr_vel_z = _inav.get_velocity_z();

    float linear_distance;  // half the distance we swap between linear and sqrt and the distance we offset sqrt
    float linear_velocity;  // the velocity we swap between linear and sqrt

    // if position controller is active add current velocity error to avoid sudden jump in acceleration
    if (is_active_z()) {
        curr_vel_z += _vel_error.z;
        if (_flags.use_desvel_ff_z) {
            curr_vel_z -= _vel_desired.z;
        }
    }

    // avoid divide by zero by using current position if kP is very low or acceleration is zero
    if (_p_pos_z.kP() <= 0.0f || _accel_z_cms <= 0.0f) {
        stopping_point.z = curr_pos_z;
        return;
    }

    // calculate the velocity at which we switch from calculating the stopping point using a linear function to a sqrt function
    linear_velocity = _accel_z_cms/_p_pos_z.kP();

    if (fabsf(curr_vel_z) < linear_velocity) {
        // if our current velocity is below the cross-over point we use a linear function
        stopping_point.z = curr_pos_z + curr_vel_z/_p_pos_z.kP();
    } else {
        linear_distance = _accel_z_cms/(2.0f*_p_pos_z.kP()*_p_pos_z.kP());
        if (curr_vel_z > 0){
            stopping_point.z = curr_pos_z + (linear_distance + curr_vel_z*curr_vel_z/(2.0f*_accel_z_cms));
        } else {
            stopping_point.z = curr_pos_z - (linear_distance + curr_vel_z*curr_vel_z/(2.0f*_accel_z_cms));
        }
    }
    stopping_point.z = constrain_float(stopping_point.z, curr_pos_z - POSCONTROL_STOPPING_DIST_DOWN_MAX, curr_pos_z + POSCONTROL_STOPPING_DIST_UP_MAX);
}

/// init_takeoff - initialises target altitude if we are taking off
void AC_PosControl::init_takeoff()
{
    // Set demanded position to current value
    const Vector3f& curr_pos = _inav.get_position();
    _pos_target.z = curr_pos.z;

    // tell the position controller to keep rotors level until takeoff is confirmed
    _taking_off = true;

    // freeze feedforward to avoid jump
    freeze_ff_z();

    // shift difference between last motor out and hover throttle into accelerometer I
    _pid_accel_z.set_integrator((_motors.get_throttle()-_motors.get_throttle_hover())*1000.0f);

    // initialise ekf reset handler
    init_ekf_z_reset();

}

// is_active_z - returns true if the z-axis position controller has been run very recently
bool AC_PosControl::is_active_z() const
{
    return ((AP_HAL::millis() - _last_update_z_ms) <= POSCONTROL_ACTIVE_TIMEOUT_MS);
}

/// update_z_controller - fly to altitude in cm above home
void AC_PosControl::update_z_controller()
{
    // check time since last cast
    uint32_t now = AP_HAL::millis();
    if (now - _last_update_z_ms > POSCONTROL_ACTIVE_TIMEOUT_MS) {
        _flags.reset_rate_to_accel_z = true;
        _flags.reset_accel_to_throttle = true;
    }
    _last_update_z_ms = now;

    // check for ekf altitude reset
    check_for_ekf_z_reset();

    // check if leash lengths need to be recalculated
    calc_leash_length_z();

    // call z-axis position controller
    calc_roll_pitch_throttle();
}

/// calc_leash_length - calculates the vertical leash lengths from maximum speed, acceleration
///     called by update_z_controller if z-axis speed or accelerations are changed
void AC_PosControl::calc_leash_length_z()
{
    if (_flags.recalc_leash_z) {
        _leash_up_z = calc_leash_length(_speed_up_cms, _accel_z_cms, _p_pos_z.kP());
        _leash_down_z = calc_leash_length(-_speed_down_cms, _accel_z_cms, _p_pos_z.kP());
        _flags.recalc_leash_z = false;
    }
}

// Calculates throttle, roll and pitch demands required to track vertical position and velocity
// and horizontal velocity and acceleration demands.
void AC_PosControl::calc_roll_pitch_throttle()
{
    float curr_alt = _inav.get_altitude();

    // clear position limit flags
    _limit.pos_up = false;
    _limit.pos_down = false;

    // calculate altitude error
    _pos_error.z = _pos_target.z - curr_alt;

    // do not let target altitude get too far from current altitude
    if (_pos_error.z > _leash_up_z) {
        _pos_target.z = curr_alt + _leash_up_z;
        _pos_error.z = _leash_up_z;
        _limit.pos_up = true;
    }
    if (_pos_error.z < -_leash_down_z) {
        _pos_target.z = curr_alt - _leash_down_z;
        _pos_error.z = -_leash_down_z;
        _limit.pos_down = true;
    }

    // calculate _vel_target.z using from _pos_error.z using sqrt controller
    _vel_target.z = AC_AttitudeControl::sqrt_controller(_pos_error.z, _p_pos_z.kP(), _accel_z_cms, _dt);

    // check speed limits
    // To-Do: check these speed limits here or in the pos->rate controller
    _limit.vel_up = false;
    _limit.vel_down = false;
    if (_vel_target.z < _speed_down_cms) {
        _vel_target.z = _speed_down_cms;
        _limit.vel_down = true;
    }
    if (_vel_target.z > _speed_up_cms) {
        _vel_target.z = _speed_up_cms;
        _limit.vel_up = true;
    }

    // add feed forward component
    if (_flags.use_desvel_ff_z) {
        _vel_target.z += _vel_desired.z;
    }

    // the following section calculates acceleration required to achieve the velocity target

    const Vector3f& curr_vel = _inav.get_velocity();

    // TODO: remove velocity derivative calculation
    // reset last velocity target to current target
    if (_flags.reset_rate_to_accel_z) {
        _vel_last.z = _vel_target.z;
    }

    // feed forward desired acceleration calculation
    if (_dt > 0.0f) {
    	if (!_flags.freeze_ff_z) {
    	    _accel_desired.z = (_vel_target.z - _vel_last.z)/_dt;
        } else {
    		// stop the feed forward being calculated during a known discontinuity
    		_flags.freeze_ff_z = false;
    	}
    } else {
        _accel_desired.z = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.z = _vel_target.z;

    // reset velocity error and filter if this controller has just been engaged
    if (_flags.reset_rate_to_accel_z) {
        // Reset Filter
        _vel_error.z = 0;
        _vel_error_filter.reset(0);
        _wing_lift_accel_filter.reset(0);
        _wing_drag_accel_filter.reset(0);
        _flags.reset_rate_to_accel_z = false;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        _vel_error.z = _vel_error_filter.apply(_vel_target.z - curr_vel.z, _dt);
    }

    _accel_target.z = _p_vel_z.get_p(_vel_error.z);

    _accel_target.z += _accel_desired.z;


    // the following section calculates a desired throttle needed to achieve the acceleration target
    float z_accel_meas;         // actual acceleration
    float p,i,d;              // used to capture pid values for logging

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;

    // reset target altitude if this controller has just been engaged
    if (_flags.reset_accel_to_throttle) {
        // Reset Filter
        _accel_error.z = 0;
        _flags.reset_accel_to_throttle = false;
    } else {
        // calculate accel error
        _accel_error.z = _accel_target.z - z_accel_meas;
    }

    // set input to PID
    _pid_accel_z.set_input_filter_all(_accel_error.z);
    _pid_accel_z.set_desired_rate(_accel_target.z);

    // separately calculate p, i, d values for logging
    p = _pid_accel_z.get_p();

    // get i term
    i = _pid_accel_z.get_integrator();

    // ensure imax is always large enough to overpower hover throttle
    if (_motors.get_throttle_hover() * 1000.0f > _pid_accel_z.imax()) {
        _pid_accel_z.imax(_motors.get_throttle_hover() * 1000.0f);
    }

    // update i term as long as we haven't breached the limits or the I term will certainly reduce
    // To-Do: should this be replaced with limits check from attitude_controller?
    if ((!_motors.limit.throttle_lower && !_motors.limit.throttle_upper) || (i>0&&_accel_error.z<0) || (i<0&&_accel_error.z>0)) {
        i = _pid_accel_z.get_i();
    }

    // get d term
    d = _pid_accel_z.get_d();

    // calculate the lift g demand scaled as an equivalent throttle
    float az_pd_gain = constrain_float(1.0f - _fwd_az_gf * _ahrs_wing.cos_pitch(), 0.1f, 1.0f);
    float lift_g_pid = (az_pd_gain * p + i + az_pd_gain * d) * 0.001f;
    float lift_g_demand = (1.0f + lift_g_pid);

    if (_taking_off) {
        // send lift demand direct to throttle and keep rotors level
        _attitude_control.set_throttle_out(lift_g_demand * _motors.get_throttle_hover(), false, POSCONTROL_THROTTLE_CUTOFF_FREQ);
        _pitch_target_cd = 0.0f;
        _roll_target_cd = 0.0f;

        // if wing has pitched up past 45 degrees, assume we have cleared the ground
        if (_ahrs_wing.cos_pitch() < 0.7071f) {
            _taking_off = false;
        }

        return;
    }

    // estimate wing force normal g in lift direction
    float wing_lift_g = _accel_z_wing_k * _ahrs_wing.cos_pitch() * _ahrs_wing.cos_pitch();
    wing_lift_g = _wing_lift_accel_filter.apply(wing_lift_g, _dt);
    wing_lift_g = constrain_float(wing_lift_g, 0.0f, 1.0f);

    // get the lift g required from the rotors taking wing lift into account
    lift_g_demand -=  wing_lift_g;
    lift_g_demand = constrain_float(lift_g_demand, -0.25f, 2.0f);

    // Logging for debug and tuning of TVBS positon controller mods
    uint32_t now = AP_HAL::millis();
    if (now - _last_log_time_ms >= 50) {
        _last_log_time_ms = now;
        DataFlash_Class::instance()->Log_Write("TVB1", "TimeUS,LGP,HT,WLG,LGD", "Qffff",
                                               AP_HAL::micros64(),
                                               (double)lift_g_pid,
                                               (double)_motors.get_throttle_hover(),
                                               (double)wing_lift_g,
                                               (double)lift_g_demand);

    }

    // calculate the throttle demand using one of two methods
    // the first combines the horizontal velocity and acceleration demand from the positon controller with the lift_g_demand
    // the second is if there is no horizontal demand, and the pilot is demanding rotor tilt directly.
    float throttle_demand;
    if (_accel_target_xy_updated) {
        _accel_target_xy_updated = false;
        // get component of velocity demand forward in wind coordinates
        float vel_forward = 0.01f * ((_vel_target.x + _vel_xy_error_integ.x) * _ahrs.cos_yaw() + (_vel_target.y + _vel_xy_error_integ.y) * _ahrs.sin_yaw());
        float alpha_coef = constrain_float(_dt / MAX(_trim_tau,0.1f),0.0f,1.0f);
        _vel_forward_filt = alpha_coef * vel_forward + (1.0f - alpha_coef) * _vel_forward_filt;

        // use forward velocity to calculate a profile drag that needs to be overcome by the rotors
        float rho = 1.225f / sqrtf(_ahrs.get_EAS2TAS());
        float fwd_g_trim = (rho / (2.0f * MAX(_fwd_bcoef, 1.0f))) * (_vel_forward_filt * _vel_forward_filt) / GRAVITY_MSS;
        if (_vel_forward_filt < 0.0f) {
            fwd_g_trim = - fwd_g_trim;
        }

        // compensate for wing normal force in the forward flight direction that needs to be overcome by the rotors
        fwd_g_trim += _accel_z_wing_k * _ahrs_wing.sin_pitch() * _ahrs_wing.cos_pitch() * _ahrs_wing.cos_roll();
        fwd_g_trim = _wing_drag_accel_filter.apply(fwd_g_trim, _dt);
        fwd_g_trim = constrain_float(fwd_g_trim, -1.0f, 1.0f);

        // rotate position controller accelerations into body forward-right frame
        float fwd_g_posctl = (_accel_target.x*_ahrs.cos_yaw() + _accel_target.y*_ahrs.sin_yaw()) / GRAVITY_CMSS;
        fwd_g_posctl = constrain_float(fwd_g_posctl, -1.0f, 1.0f);
        float right_g_posctl = (-_accel_target.x*_ahrs.sin_yaw() + _accel_target.y*_ahrs.cos_yaw()) / GRAVITY_CMSS;
        right_g_posctl = constrain_float(right_g_posctl, -1.0f, 1.0f);


        // Check if a touchdown is expected and if detected, set fwd and lateral g demnds to zero
        if ((now - _land_expect_time_ms) < 1000) {
            // wing pitch rate on touchdown
            float pitch_rate_mag = fabsf(_ahrs_wing.get_gyro_latest().y);
            if (pitch_rate_mag > _landed_gyro_y) {
                _land_pitch_rate_time_ms = now;
            }

            // wing within touchdown pitch angle range
            float wing_pitch_ang = _ahrs_wing.pitch;
            if (wing_pitch_ang < _landed_pitch) {
                _land_pitch_ang_time_ms = now;
            }

            // landing shock detected.
            float accel_mag = _ahrs_wing.get_accel_ef_blended().length();
            if (accel_mag > _landed_accel) {
                _land_accz_time_ms = now;
            }

            // declare landing detected if 2 out of 3 criteria were met in the last second
            // only require one check if already detected
            uint8_t counter = 0;
            if ((now - _land_pitch_rate_time_ms) < 1000) {
                counter++;
            }
            if ((now - _land_pitch_ang_time_ms) < 1000) {
                counter++;
            }
            if ((now - _land_accz_time_ms) < 1000) {
                counter++;
            }
            if ((counter >= 2) || ((counter >= 1) && _is_landed)) {
                _is_landed = true;
                _land_detect_time_ms = now;
            } else {
                _is_landed = false;
            }

            // if landing detected, zero horizontal g maneouvre demands to point rotors up and prevent horizontal dragging and prop strike
            if ((now - _land_detect_time_ms) < 3000) {
                fwd_g_trim = fwd_g_posctl = right_g_posctl = 0.0f;
            }

            DataFlash_Class::instance()->Log_Write("VLND", "TimeUS,GYRY,PITCH,ACCEL,COUNT", "QfffB",
                                                       AP_HAL::micros64(),
                                                       (double)pitch_rate_mag,
                                                       (double)wing_pitch_ang,
                                                       (double)accel_mag,
                                                       (uint8_t)counter);
        } else {
            _land_detect_time_ms = 0;
            _land_pitch_rate_time_ms = 0;
            _land_pitch_ang_time_ms = 0;
            _land_accz_time_ms = 0;
            _is_landed = false;
        }

        // Combine fwd and vertical g demands to obtain the required thrust g vector. Prevent loss of control due to
        // reverse flow during braking maneouvres by waiting until g due to aero drag has fallen before allowing
        // rotors to tilt past vertical.
        float fwd_g_demand = fwd_g_trim + fwd_g_posctl;
        if (fwd_g_trim > _drag_brake_g_thrhld) {
            fwd_g_demand = fmaxf(fwd_g_demand, 0.0f);
        } else if (fwd_g_trim < -_drag_brake_g_thrhld) {
            fwd_g_demand = fminf(fwd_g_demand, 0.0f);
        }
        float pitch_target_rad = atan2f(-fwd_g_demand , lift_g_demand);
        float thrust_g_demand = sqrtf(fwd_g_demand * fwd_g_demand + lift_g_demand * lift_g_demand);

        // Limit the pitch target
        float min_pitch_angle = -radians(_attitude_control.lean_angle_max_fwd());
        float max_pitch_angle = radians(_attitude_control.lean_angle_max_aft());
        pitch_target_rad = constrain_float(pitch_target_rad, min_pitch_angle, max_pitch_angle);

        // calculate throttle required to generate thrust
        // rough correction for prop inflow is included - needs refinement
        float vel_inflow = MAX(- _vel_forward_filt * sinf(pitch_target_rad), 0.0f);
        float thrust_g_inflow = _fwd_inflow_thrust_factor * vel_inflow;
        throttle_demand = constrain_float((thrust_g_demand + thrust_g_inflow) * _motors.get_throttle_hover(), 0.0f, 1.0f);

        // rotate the thrust vector and adjust the magnitude to maintain lift and achieve the required forward acceleration
        // calculate the roll assuming only rotor  provides significant force in that direction
        float cos_pitch_target = cosf(pitch_target_rad);
        _pitch_target_cd = 100.0f *  degrees(pitch_target_rad);
        _roll_target_cd = degrees(atanf(right_g_posctl * cos_pitch_target)) ;
        _roll_target_cd = 100.0f * constrain_float(_roll_target_cd, -_attitude_control.lean_angle_max_lat(), _attitude_control.lean_angle_max_lat());


        // send throttle to attitude controller without angle boost
        _attitude_control.set_throttle_out(throttle_demand, false, POSCONTROL_THROTTLE_CUTOFF_FREQ);

        // Logging for debug and tuning of TVBS positon controller mods
        if (now - _last_log_time_ms >= 50 || now == _last_log_time_ms) {
            _last_log_time_ms = now;

            DataFlash_Class::instance()->Log_Write("TVB2", "TimeUS,VXI,VYI,VFF,FGP,RGP,FGT,TGD,TGI,PTC,RTC", "Qffffffffff",
                                                   AP_HAL::micros64(),
                                                   (double)(0.01f*_vel_xy_error_integ.x),
                                                   (double)(0.01f*_vel_xy_error_integ.y),
                                                   (double)_vel_forward_filt,
                                                   (double)fwd_g_posctl,
                                                   (double)right_g_posctl,
                                                   (double)fwd_g_trim,
                                                   (double)thrust_g_demand,
                                                   (float)thrust_g_inflow,
                                                   (double)_pitch_target_cd,
                                                   (double)_roll_target_cd);

            // write generic multicopter position control message
            write_log();
        }

    } else {
        // multiply by hover throttle (only works properly when motors are pointing up)
        throttle_demand = lift_g_demand * _motors.get_throttle_hover();

        // send throttle to attitude controller with angle boost
        _attitude_control.set_throttle_out(throttle_demand, true, POSCONTROL_THROTTLE_CUTOFF_FREQ);

    }
}

///
/// lateral position controller
///

/// set_max_accel_xy - set the maximum horizontal acceleration in cm/s/s
void AC_PosControl::set_max_accel_xy(float accel_cmss)
{
    if (fabsf(_accel_cms-accel_cmss) > 1.0f) {
        _accel_cms = accel_cmss;
        _flags.recalc_leash_xy = true;
        calc_leash_length_xy();
    }
}

/// set_max_speed_xy - set the maximum horizontal speed maximum in cm/s
void AC_PosControl::set_max_speed_xy(float speed_cms)
{
    if (fabsf(_speed_cms-speed_cms) > 1.0f) {
        _speed_cms = speed_cms;
        _flags.recalc_leash_xy = true;
        calc_leash_length_xy();
    }
}

/// set_pos_target in cm from home
void AC_PosControl::set_pos_target(const Vector3f& position)
{
    _pos_target = position;

    _flags.use_desvel_ff_z = false;
    _vel_desired.z = 0.0f;
    // initialise roll and pitch to current roll and pitch.  This avoids a twitch between when the target is set and the pos controller is first run
    // To-Do: this initialisation of roll and pitch targets needs to go somewhere between when pos-control is initialised and when it completes it's first cycle
    //_roll_target = constrain_int32(_ahrs.roll_sensor,-_attitude_control.lean_angle_max(),_attitude_control.lean_angle_max());
    //_pitch_target = constrain_int32(_ahrs.pitch_sensor,-_attitude_control.lean_angle_max(),_attitude_control.lean_angle_max());
}

/// set_xy_target in cm from home
void AC_PosControl::set_xy_target(float x, float y)
{
    _pos_target.x = x;
    _pos_target.y = y;
}

/// shift position target target in x, y axis
void AC_PosControl::shift_pos_xy_target(float x_cm, float y_cm)
{
    // move pos controller target
    _pos_target.x += x_cm;
    _pos_target.y += y_cm;
}

/// set_target_to_stopping_point_xy - sets horizontal target to reasonable stopping position in cm from home
void AC_PosControl::set_target_to_stopping_point_xy()
{
    // check if xy leash needs to be recalculated
    calc_leash_length_xy();

    get_stopping_point_xy(_pos_target);
}

/// get_stopping_point_xy - calculates stopping point based on current position, velocity, vehicle acceleration
///     distance_max allows limiting distance to stopping point
///     results placed in stopping_position vector
///     set_max_accel_xy() should be called before this method to set vehicle acceleration
///     set_leash_length() should have been called before this method
void AC_PosControl::get_stopping_point_xy(Vector3f &stopping_point) const
{
    const Vector3f curr_pos = _inav.get_position();
    Vector3f curr_vel = _inav.get_velocity();
    float linear_distance;      // the distance at which we swap from a linear to sqrt response
    float linear_velocity;      // the velocity above which we swap from a linear to sqrt response
    float stopping_dist;		// the distance within the vehicle can stop
    float kP = _p_pos_xy.kP();

    // add velocity error to current velocity
    if (is_active_xy()) {
        curr_vel.x += _vel_error.x;
        curr_vel.y += _vel_error.y;
    }

    // calculate current velocity
    float vel_total = norm(curr_vel.x, curr_vel.y);

    // avoid divide by zero by using current position if the velocity is below 10cm/s, kP is very low or acceleration is zero
    if (kP <= 0.0f || _accel_cms <= 0.0f || is_zero(vel_total)) {
        stopping_point.x = curr_pos.x;
        stopping_point.y = curr_pos.y;
        return;
    }

    // calculate point at which velocity switches from linear to sqrt
    linear_velocity = _accel_cms/kP;

    // calculate distance within which we can stop
    if (vel_total < linear_velocity) {
    	stopping_dist = vel_total/kP;
    } else {
        linear_distance = _accel_cms/(2.0f*kP*kP);
        stopping_dist = linear_distance + (vel_total*vel_total)/(2.0f*_accel_cms);
    }

    // constrain stopping distance
    stopping_dist = constrain_float(stopping_dist, 0, _leash);

    // convert the stopping distance into a stopping point using velocity vector
    stopping_point.x = curr_pos.x + (stopping_dist * curr_vel.x / vel_total);
    stopping_point.y = curr_pos.y + (stopping_dist * curr_vel.y / vel_total);
}

/// get_distance_to_target - get horizontal distance to target position in cm
float AC_PosControl::get_distance_to_target() const
{
    return norm(_pos_error.x, _pos_error.y);
}

/// get_bearing_to_target - get bearing to target position in centi-degrees
int32_t AC_PosControl::get_bearing_to_target() const
{
    return get_bearing_cd(_inav.get_position(), _pos_target);
}

// is_active_xy - returns true if the xy position controller has been run very recently
bool AC_PosControl::is_active_xy() const
{
    return ((AP_HAL::millis() - _last_update_xy_ms) <= POSCONTROL_ACTIVE_TIMEOUT_MS);
}

/// get_lean_angle_max_cd - returns the maximum lean angle the autopilot may request
float AC_PosControl::get_lean_angle_max_cd() const
{
    return 100.0f * MIN(MIN(_attitude_control.lean_angle_max_fwd(), _attitude_control.lean_angle_max_aft()), _attitude_control.lean_angle_max_lat());
}

/// init_xy_controller - initialise the xy controller
///     this should be called after setting the position target and the desired velocity and acceleration
///     sets target roll angle, pitch angle and I terms based on vehicle current lean angles
///     should be called once whenever significant changes to the position target are made
///     this does not update the xy target
void AC_PosControl::init_xy_controller()
{
    // set roll, pitch lean angle targets to current attitude
    // todo: this should probably be based on the desired attitude not the current attitude
    _roll_target_cd = _ahrs.roll_sensor;
    _pitch_target_cd = _ahrs.pitch_sensor;

    // initialise I terms from lean angles
    _pid_vel_xy.reset_filter();
    lean_angles_to_accel(_accel_target.x, _accel_target.y);
    _pid_vel_xy.set_integrator(_accel_target-_accel_desired);

    // flag reset required in rate to accel step
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_accel_to_lean_xy = true;

    // initialise ekf xy reset handler
    init_ekf_xy_reset();

    // Set velocity error integrator to reciprocal of estimated wind speed
    Vector3f wind_vec = _ahrs_wing.wind_estimate();
    _vel_xy_error_integ.x = -100.0f * wind_vec.x;
    _vel_xy_error_integ.y = -100.0f * wind_vec.y;

    // Reset remaining horizontal control states
    _vel_xy_integ_length_prev = norm(_vel_xy_error_integ.x, _vel_xy_error_integ.y);
    _vel_error_filter.reset(0);
    _accel_target_xy_updated = false;
    _vel_forward_filt = 0.0f;
    _last_log_time_ms = 0;

}

/// update_xy_controller - run the horizontal position controller - should be called at 100hz or higher
void AC_PosControl::update_xy_controller()
{
    // compute dt
    uint32_t now = AP_HAL::millis();
    float dt = (now - _last_update_xy_ms)*0.001f;

    // sanity check dt
    if (dt >= POSCONTROL_ACTIVE_TIMEOUT_MS*1.0e-3f) {
        dt = 0.0f;

        // Set velocity error integrator to reciprocal of estimated wind speed
        Vector3f wind_vec = _ahrs_wing.wind_estimate();
        _vel_xy_error_integ.x = -100.0f * wind_vec.x;
        _vel_xy_error_integ.y = -100.0f * wind_vec.y;

        // Reset required horizontal control states
        _vel_xy_integ_length_prev = norm(_vel_xy_error_integ.x, _vel_xy_error_integ.y);
        _vel_error_filter.reset(0);
        _accel_target_xy_updated = false;
        _vel_forward_filt = 0.0f;
        _last_log_time_ms = 0;

    }

    // check for ekf xy position reset
    check_for_ekf_xy_reset();

    // check if xy leash needs to be recalculated
    calc_leash_length_xy();

    // translate any adjustments from pilot to loiter target
    desired_vel_to_pos(dt);

    // run horizontal position controller
    run_xy_controller(dt);

    // update xy update time
    _last_update_xy_ms = now;
}

float AC_PosControl::time_since_last_xy_update() const
{
    uint32_t now = AP_HAL::millis();
    return (now - _last_update_xy_ms)*0.001f;
}

// write log to dataflash
void AC_PosControl::write_log()
{
    const Vector3f &pos_target = get_pos_target();
    const Vector3f &vel_target = get_vel_target();
    const Vector3f &accel_target = get_accel_target();
    const Vector3f &position = _inav.get_position();
    const Vector3f &velocity = _inav.get_velocity();
    float accel_x, accel_y;
    lean_angles_to_accel(accel_x, accel_y);

    DataFlash_Class::instance()->Log_Write("PSC", "TimeUS,TPX,TPY,PX,PY,TVX,TVY,VX,VY,TAX,TAY,AX,AY",
                                           "smmmmnnnnoooo", "FBBBBBBBBBBBB", "Qffffffffffff",
                                           AP_HAL::micros64(),
                                           (double)pos_target.x,
                                           (double)pos_target.y,
                                           (double)position.x,
                                           (double)position.y,
                                           (double)vel_target.x,
                                           (double)vel_target.y,
                                           (double)velocity.x,
                                           (double)velocity.y,
                                           (double)accel_target.x,
                                           (double)accel_target.y,
                                           (double)accel_x,
                                           (double)accel_y);

}

/// init_vel_controller_xyz - initialise the velocity controller - should be called once before the caller attempts to use the controller
void AC_PosControl::init_vel_controller_xyz()
{
    // set roll, pitch lean angle targets to current attitude
    _roll_target_cd = _ahrs.roll_sensor;
    _pitch_target_cd = _ahrs.pitch_sensor;

    _pid_vel_xy.reset_filter();
    lean_angles_to_accel(_accel_target.x, _accel_target.y);
    _pid_vel_xy.set_integrator(_accel_target);

    // flag reset required in rate to accel step
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_accel_to_lean_xy = true;

    // set target position
    const Vector3f& curr_pos = _inav.get_position();
    set_xy_target(curr_pos.x, curr_pos.y);
    set_alt_target(curr_pos.z);

    // move current vehicle velocity into feed forward velocity
    const Vector3f& curr_vel = _inav.get_velocity();
    set_desired_velocity(curr_vel);

    // set vehicle acceleration to zero
    set_desired_accel_xy(0.0f,0.0f);

    // initialise ekf reset handlers
    init_ekf_xy_reset();
    init_ekf_z_reset();
}

/// update_velocity_controller_xy - run the velocity controller - should be called at 100hz or higher
///     velocity targets should we set using set_desired_velocity_xy() method
///     callers should use get_roll() and get_pitch() methods and sent to the attitude controller
///     throttle targets will be sent directly to the motors
void AC_PosControl::update_vel_controller_xy()
{
    // capture time since last iteration
    uint32_t now = AP_HAL::millis();
    float dt = (now - _last_update_xy_ms)*0.001f;

    // sanity check dt
    if (dt >= 0.2f) {
        dt = 0.0f;
    }

    // check for ekf xy position reset
    check_for_ekf_xy_reset();

    // check if xy leash needs to be recalculated
    calc_leash_length_xy();

    // apply desired velocity request to position target
    // TODO: this will need to be removed and added to the calling function.
    desired_vel_to_pos(dt);

    // run position controller
    run_xy_controller(dt);

    // update xy update time
    _last_update_xy_ms = now;
}


/// update_velocity_controller_xyz - run the velocity controller - should be called at 100hz or higher
///     velocity targets should we set using set_desired_velocity_xyz() method
///     callers should use get_roll() and get_pitch() methods and sent to the attitude controller
///     throttle targets will be sent directly to the motors
void AC_PosControl::update_vel_controller_xyz()
{
    update_vel_controller_xy();

    // update altitude target
    set_alt_target_from_climb_rate_ff(_vel_desired.z, _dt, false);

    // run z-axis position controller
    update_z_controller();
}

float AC_PosControl::get_horizontal_error() const
{
    return norm(_pos_error.x, _pos_error.y);
}

///
/// private methods
///

/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration
///     should be called whenever the speed, acceleration or position kP is modified
void AC_PosControl::calc_leash_length_xy()
{
    // todo: remove _flags.recalc_leash_xy or don't call this function after each variable change.
    if (_flags.recalc_leash_xy) {
        _leash = calc_leash_length(_speed_cms, _accel_cms, _p_pos_xy.kP());
        _flags.recalc_leash_xy = false;
    }
}

/// move velocity target using desired acceleration
void AC_PosControl::desired_accel_to_vel(float nav_dt)
{
    // range check nav_dt
    if (nav_dt < 0) {
        return;
    }

    // update target velocity
    if (_flags.reset_desired_vel_to_pos) {
        _flags.reset_desired_vel_to_pos = false;
    } else {
        _vel_desired.x += _accel_desired.x * nav_dt;
        _vel_desired.y += _accel_desired.y * nav_dt;
    }
}

/// desired_vel_to_pos - move position target using desired velocities
void AC_PosControl::desired_vel_to_pos(float nav_dt)
{
    // range check nav_dt
    if( nav_dt < 0 ) {
        return;
    }

    // update target position
    if (_flags.reset_desired_vel_to_pos) {
        _flags.reset_desired_vel_to_pos = false;
    } else {
        _pos_target.x += _vel_desired.x * nav_dt;
        _pos_target.y += _vel_desired.y * nav_dt;
    }
}

/// run horizontal position controller correcting position and velocity
///     converts position (_pos_target) to target velocity (_vel_target)
///     desired velocity (_vel_desired) is combined into final target velocity
///     converts desired velocities in lat/lon directions to accelerations in lat/lon frame
///     converts desired accelerations provided in lat/lon frame to roll/pitch angles
void AC_PosControl::run_xy_controller(float dt)
{
    float ekfGndSpdLimit, ekfNavVelGainScaler;
    AP::ahrs_navekf().getEkfControlLimits(ekfGndSpdLimit, ekfNavVelGainScaler);

    Vector3f curr_pos = _inav.get_position();
    float kP = ekfNavVelGainScaler * _p_pos_xy.kP(); // scale gains to compensate for noisy optical flow measurement in the EKF

    // avoid divide by zero
    if (kP <= 0.0f) {
        _vel_target.x = 0.0f;
        _vel_target.y = 0.0f;
    }else{
        // calculate distance error
        _pos_error.x = _pos_target.x - curr_pos.x;
        _pos_error.y = _pos_target.y - curr_pos.y;

        // Constrain _pos_error and target position
        // Constrain the maximum length of _vel_target to the maximum position correction velocity
        // TODO: replace the leash length with a user definable maximum position correction
        if (limit_vector_length(_pos_error.x, _pos_error.y, _leash))
        {
            _pos_target.x = curr_pos.x + _pos_error.x;
            _pos_target.y = curr_pos.y + _pos_error.y;
        }

        _vel_target = sqrt_controller(_pos_error, kP, _accel_cms);
    }

    // add velocity feed-forward
    _vel_target.x += _vel_desired.x;
    _vel_target.y += _vel_desired.y;

    // the following section converts desired velocities in lat/lon directions to accelerations in lat/lon frame

    Vector2f accel_target, vel_xy_p, vel_xy_i, vel_xy_d;

    // check if vehicle velocity is being overridden
    if (_flags.vehicle_horiz_vel_override) {
        _flags.vehicle_horiz_vel_override = false;
    } else {
        _vehicle_horiz_vel.x = _inav.get_velocity().x;
        _vehicle_horiz_vel.y = _inav.get_velocity().y;
    }

    // calculate velocity error
    _vel_error.x = _vel_target.x - _vehicle_horiz_vel.x;
    _vel_error.y = _vel_target.y - _vehicle_horiz_vel.y;
    // TODO: constrain velocity error and velocity target

    // calculate integral of velocity error and constrain
    // integrator gain can be scaled externally, but asymptotes back to unity over a 1 second time constant if not updated
    _vel_err_i_gain_scale = constrain_float((1.0f - dt) * _vel_err_i_gain_scale + dt, 1.0f , 10.0f);
    _vel_xy_error_integ.x += _vel_err_i_gain_scale * _vel_err_i_gain * _vel_error.x * dt;
    _vel_xy_error_integ.y += _vel_err_i_gain_scale * _vel_err_i_gain * _vel_error.y * dt;
    float _vel_xy_error_integ_norm = norm(_vel_xy_error_integ.x, _vel_xy_error_integ.y);
    float _max_airspeed_cms = 100.0f * _fwd_spd_max;
    if (_vel_xy_error_integ_norm > _max_airspeed_cms) {
        _vel_xy_error_integ = _vel_xy_error_integ * (_max_airspeed_cms / _vel_xy_error_integ_norm);
    }
    if (!_limit.accel_xy && !_motors.limit.throttle_upper) {
        _vel_xy_integ_length_prev = norm(_vel_xy_error_integ.x, _vel_xy_error_integ.y);
    } else if (norm(_vel_xy_error_integ.x, _vel_xy_error_integ.y) > _vel_xy_integ_length_prev) {
        _vel_xy_error_integ = _vel_xy_error_integ * (_vel_xy_integ_length_prev / _vel_xy_error_integ_norm);
    }

    // call pi controller
    _pid_vel_xy.set_input(_vel_error);

    // get p
    vel_xy_p = _pid_vel_xy.get_p();

    // update i term if we have not hit the accel or throttle limits OR the i term will reduce
    // TODO: move limit handling into the PI and PID controller
    if (!_limit.accel_xy && !_motors.limit.throttle_upper) {
        vel_xy_i = _pid_vel_xy.get_i();
    } else {
        vel_xy_i = _pid_vel_xy.get_i_shrink();
    }

    // get d
    vel_xy_d = _pid_vel_xy.get_d();

    // acceleration to correct for velocity error and scale PID output to compensate for optical flow measurement induced EKF noise
    accel_target.x = (vel_xy_p.x + vel_xy_i.x + vel_xy_d.x) * ekfNavVelGainScaler;
    accel_target.y = (vel_xy_p.y + vel_xy_i.y + vel_xy_d.y) * ekfNavVelGainScaler;

    // reset accel to current desired acceleration
     if (_flags.reset_accel_to_lean_xy) {
         _accel_target_filter.reset(Vector2f(accel_target.x, accel_target.y));
         _flags.reset_accel_to_lean_xy = false;
     }

     // filter correction acceleration
    _accel_target_filter.set_cutoff_frequency(MIN(_accel_xy_filt_hz, 5.0f*ekfNavVelGainScaler));
    _accel_target_filter.apply(accel_target, dt);

    // pass the correction acceleration to the target acceleration output
    _accel_target.x = _accel_target_filter.get().x;
    _accel_target.y = _accel_target_filter.get().y;

    // Add feed forward into the target acceleration output
    _accel_target.x += _accel_desired.x;
    _accel_target.y += _accel_desired.y;

    // limit acceleration
    _limit.accel_xy = limit_vector_length(_accel_target.x, _accel_target.y, POSCONTROL_ACCEL_XY_MAX);

    _accel_target_xy_updated = true;

}

void AC_PosControl::reset_wind_drift_integ()
{
    float vel_forward = 0.01f * (_vehicle_horiz_vel.x * _ahrs.cos_yaw() + _vehicle_horiz_vel.y * _ahrs.sin_yaw());
    float vel_forward_diff = _vel_forward_filt - vel_forward;
    _vel_xy_error_integ.x += 100.0f * vel_forward_diff * _ahrs.cos_yaw();
    _vel_xy_error_integ.y += 100.0f * vel_forward_diff * _ahrs.sin_yaw();
    float _vel_xy_error_integ_norm = norm(_vel_xy_error_integ.x, _vel_xy_error_integ.y);
    float _max_airspeed_cms = 100.0f * _fwd_spd_max;
    if (_vel_xy_error_integ_norm > _max_airspeed_cms) {
        _vel_xy_error_integ = _vel_xy_error_integ * (_max_airspeed_cms / _vel_xy_error_integ_norm);
    }
}

// get_lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
void AC_PosControl::accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float& roll_target, float& pitch_target) const
{
    float accel_right, accel_forward;

    // rotate accelerations into body forward-right frame
    // todo: this should probably be based on the desired heading not the current heading
    accel_forward = accel_x_cmss*_ahrs.cos_yaw() + accel_y_cmss*_ahrs.sin_yaw();
    accel_right = -accel_x_cmss*_ahrs.sin_yaw() + accel_y_cmss*_ahrs.cos_yaw();

    // update angle targets that will be passed to stabilize controller
    pitch_target = atanf(-accel_forward/(GRAVITY_MSS * 100.0f))*(18000.0f/M_PI);
    float cos_pitch_target = cosf(pitch_target*M_PI/18000.0f);
    roll_target = atanf(accel_right*cos_pitch_target/(GRAVITY_MSS * 100.0f))*(18000.0f/M_PI);
}

// get_lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
void AC_PosControl::lean_angles_to_accel(float& accel_x_cmss, float& accel_y_cmss) const
{
    // rotate our roll, pitch angles into lat/lon frame
    // todo: this should probably be based on the desired attitude not the current attitude
    accel_x_cmss = (GRAVITY_MSS * 100) * (-_ahrs.cos_yaw() * _ahrs.sin_pitch() * _ahrs.cos_roll() - _ahrs.sin_yaw() * _ahrs.sin_roll()) / MAX(_ahrs.cos_roll()*_ahrs.cos_pitch(), 0.5f);
    accel_y_cmss = (GRAVITY_MSS * 100) * (-_ahrs.sin_yaw() * _ahrs.sin_pitch() * _ahrs.cos_roll() + _ahrs.cos_yaw() * _ahrs.sin_roll()) / MAX(_ahrs.cos_roll()*_ahrs.cos_pitch(), 0.5f);
}

/// calc_leash_length - calculates the horizontal leash length given a maximum speed, acceleration and position kP gain
float AC_PosControl::calc_leash_length(float speed_cms, float accel_cms, float kP) const
{
    float leash_length;

    // sanity check acceleration and avoid divide by zero
    if (accel_cms <= 0.0f) {
        accel_cms = POSCONTROL_ACCELERATION_MIN;
    }

    // avoid divide by zero
    if (kP <= 0.0f) {
        return POSCONTROL_LEASH_LENGTH_MIN;
    }

    // calculate leash length
    if(speed_cms <= accel_cms / kP) {
        // linear leash length based on speed close in
        leash_length = speed_cms / kP;
    }else{
        // leash length grows at sqrt of speed further out
        leash_length = (accel_cms / (2.0f*kP*kP)) + (speed_cms*speed_cms / (2.0f*accel_cms));
    }

    // ensure leash is at least 1m long
    if( leash_length < POSCONTROL_LEASH_LENGTH_MIN ) {
        leash_length = POSCONTROL_LEASH_LENGTH_MIN;
    }

    return leash_length;
}

/// initialise ekf xy position reset check
void AC_PosControl::init_ekf_xy_reset()
{
    Vector2f pos_shift;
    _ekf_xy_reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift);
}

/// check for ekf position reset and adjust loiter or brake target position
void AC_PosControl::check_for_ekf_xy_reset()
{
    // check for position shift
    Vector2f pos_shift;
    uint32_t reset_ms = _ahrs.getLastPosNorthEastReset(pos_shift);
    if (reset_ms != _ekf_xy_reset_ms) {
        shift_pos_xy_target(pos_shift.x * 100.0f, pos_shift.y * 100.0f);
        _ekf_xy_reset_ms = reset_ms;
    }
}

/// initialise ekf z axis reset check
void AC_PosControl::init_ekf_z_reset()
{
    float alt_shift;
    _ekf_z_reset_ms = _ahrs.getLastPosDownReset(alt_shift);
}

/// check for ekf position reset and adjust loiter or brake target position
void AC_PosControl::check_for_ekf_z_reset()
{
    // check for position shift
    float alt_shift;
    uint32_t reset_ms = _ahrs.getLastPosDownReset(alt_shift);
    if (reset_ms != 0 && reset_ms != _ekf_z_reset_ms) {
        shift_alt_target(-alt_shift * 100.0f);
        _ekf_z_reset_ms = reset_ms;
    }
}


/// limit vector to a given length, returns true if vector was limited
bool AC_PosControl::limit_vector_length(float& vector_x, float& vector_y, float max_length)
{
    float vector_length = norm(vector_x, vector_y);
    if ((vector_length > max_length) && is_positive(vector_length)) {
        vector_x *= (max_length / vector_length);
        vector_y *= (max_length / vector_length);
        return true;
    }
    return false;
}


/// Proportional controller with piecewise sqrt sections to constrain second derivative
Vector3f AC_PosControl::sqrt_controller(const Vector3f& error, float p, float second_ord_lim)
{
    if (second_ord_lim < 0.0f || is_zero(second_ord_lim) || is_zero(p)) {
        return Vector3f(error.x*p, error.y*p, error.z);
    }

    float linear_dist = second_ord_lim/sq(p);
    float error_length = norm(error.x, error.y);
    if (error_length > linear_dist) {
        float first_order_scale = safe_sqrt(2.0f*second_ord_lim*(error_length-(linear_dist * 0.5f)))/error_length;
        return Vector3f(error.x*first_order_scale, error.y*first_order_scale, error.z);
    } else {
        return Vector3f(error.x*p, error.y*p, error.z);
    }
}
