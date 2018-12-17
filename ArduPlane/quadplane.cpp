#include "Plane.h"

const AP_Param::GroupInfo QuadPlane::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable QuadPlane
    // @Description: This enables QuadPlane functionality, assuming multicopter motors start on output 5. If this is set to 2 then when starting AUTO mode it will initially be in VTOL AUTO mode.
    // @Values: 0:Disable,1:Enable,2:Enable VTOL AUTO
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLE", 1, QuadPlane, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Group: M_
    // @Path: ../libraries/AP_Motors/AP_MotorsMulticopter.cpp
    AP_SUBGROUPVARPTR(motors, "M_", 2, QuadPlane, plane.quadplane.motors_var_info),

    // 3 ~ 8 were used by quadplane attitude control PIDs

    // @Param: ANGLE_MAX
    // @DisplayName: Angle Max
    // @Description: Maximum lean angle in all VTOL flight modes
    // @Units: cdeg
    // @Range: 1000 8000
    // @User: Advanced
    AP_GROUPINFO("ANGLE_MAX", 10, QuadPlane, aparm.angle_max, 3000),

    // @Param: TRANSITION_MS
    // @DisplayName: Transition time
    // @Description: Transition time in milliseconds after minimum airspeed is reached
    // @Units: ms
    // @Range: 0 30000
    // @User: Advanced
    AP_GROUPINFO("TRANSITION_MS", 11, QuadPlane, transition_time_ms, 5000),

    // 12 ~ 16 were used by position, velocity and acceleration PIDs

    // @Group: P_
    // @Path: ../libraries/AC_AttitudeControl/AC_PosControl.cpp
    AP_SUBGROUPPTR(pos_control, "P", 17, QuadPlane, AC_PosControl),

    // @Param: VELZ_MAX
    // @DisplayName: Pilot maximum vertical speed
    // @Description: The maximum vertical velocity the pilot may request in cm/s
    // @Units: cm/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("VELZ_MAX", 18, QuadPlane, pilot_velocity_z_max, 250),

    // @Param: ACCEL_Z
    // @DisplayName: Pilot vertical acceleration
    // @Description: The vertical acceleration used when pilot is controlling the altitude
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_Z",  19, QuadPlane, pilot_accel_z,  250),

    // @Group: WP_
    // @Path: ../libraries/AC_WPNav/AC_WPNav.cpp
    AP_SUBGROUPPTR(wp_nav, "WP_",  20, QuadPlane, AC_WPNav),

    // @Param: RC_SPEED
    // @DisplayName: RC output speed in Hz
    // @Description: This is the PWM refresh rate in Hz for QuadPlane quad motors
    // @Units: Hz
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("RC_SPEED", 21, QuadPlane, rc_speed, 490),

    // @Param: THR_MIN_PWM
    // @DisplayName: Minimum PWM output
    // @Description: This is the minimum PWM output for the quad motors
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MIN_PWM", 22, QuadPlane, thr_min_pwm, 1000),

    // @Param: THR_MAX_PWM
    // @DisplayName: Maximum PWM output
    // @Description: This is the maximum PWM output for the quad motors
    // @Units: PWM
    // @Range: 800 2200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THR_MAX_PWM", 23, QuadPlane, thr_max_pwm, 2000),

    // @Param: ASSIST_SPEED
    // @DisplayName: Quadplane assistance speed
    // @Description: This is the speed below which the quad motors will provide stability and lift assistance in fixed wing modes. Zero means no assistance except during transition
    // @Units: m/s
    // @Range: 0 100
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("ASSIST_SPEED", 24, QuadPlane, assist_speed, 0),

    // @Param: YAW_RATE_MAX
    // @DisplayName: Maximum yaw rate
    // @Description: This is the maximum yaw rate for pilot input on rudder stick in degrees/second
    // @Units: deg/s
    // @Range: 50 500
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_RATE_MAX", 25, QuadPlane, yaw_rate_max, 100),

    // @Param: LAND_SPEED
    // @DisplayName: Land speed
    // @Description: The descent speed for the final stage of landing in cm/s
    // @Units: cm/s
    // @Range: 30 200
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("LAND_SPEED", 26, QuadPlane, land_speed_cms, 50),

    // @Param: LAND_FINAL_ALT
    // @DisplayName: Land final altitude
    // @Description: The altitude at which we should switch to Q_LAND_SPEED descent rate
    // @Units: m
    // @Range: 0.5 50
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("LAND_FINAL_ALT", 27, QuadPlane, land_final_alt, 6),

    // 28 was used by THR_MID

    // @Param: TRAN_PIT_MAX
    // @DisplayName: Transition max pitch
    // @Description: Maximum pitch during transition to auto fixed wing flight
    // @User: Standard
    // @Range: 0 30
    // @Units: deg
    // @Increment: 1
    AP_GROUPINFO("TRAN_PIT_MAX", 29, QuadPlane, transition_pitch_max, 3),

    // frame class was moved from 30 when consolidating AP_Motors classes
#define FRAME_CLASS_OLD_IDX 30
    // @Param: FRAME_CLASS
    // @DisplayName: Frame Class
    // @Description: Controls major frame class for multicopter component
    // @Values: 0:Undefined, 1:Quad, 2:Hexa, 3:Octa, 4:OctaQuad, 5:Y6, 7:Tri, 10: TailSitter, 14: TVBS
    // @User: Standard
    AP_GROUPINFO("FRAME_CLASS", 46, QuadPlane, frame_class, 1),

    // @Param: FRAME_TYPE
    // @DisplayName: Frame Type (+, X or V)
    // @Description: Controls motor mixing for multicopter component
    // @Values: 0:Plus, 1:X, 2:V, 3:H, 4:V-Tail, 5:A-Tail, 10:Y6B, 11:Y6F
    // @User: Standard
    AP_GROUPINFO("FRAME_TYPE", 31, QuadPlane, frame_type, 1),

    // @Param: VFWD_GAIN
    // @DisplayName: Forward velocity hold gain
    // @Description: Controls use of forward motor in vtol modes. If this is zero then the forward motor will not be used for position control in VTOL modes. A value of 0.05 is a good place to start if you want to use the forward motor for position control. No forward motor will be used in QSTABILIZE or QHOVER modes. Use QLOITER for position hold with the forward motor.
    // @Range: 0 0.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("VFWD_GAIN", 32, QuadPlane, vel_forward.gain, 0),

    // @Param: WVANE_GAIN
    // @DisplayName: Weathervaning gain
    // @Description: This controls the tendency to yaw to face into the wind. A value of 1.0 is to start with and will give a slow turn into the wind. Use a value of 10.0 for a rapid response. The weathervaning works by turning into the direction of lateral specific force. Negative values can be used to point the top of the wing to the wind for tailsitter or TVBS vehicles that hover better inverted.
    // @Range: -1.0 1.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("WVANE_GAIN", 33, QuadPlane, weathervane.gain, 0),

    // @Param: WVANE_MINROLL
    // @DisplayName: Weathervaning min roll
    // @Description: This set the minimum roll in degrees before active weathervaning will start. This may need to be larger if your aircraft has bad roll trim.
    // @Range: 0 10
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("WVANE_MINROLL", 34, QuadPlane, weathervane.min_roll, 1),

    // @Param: RTL_ALT
    // @DisplayName: QRTL return altitude
    // @Description: The altitude which QRTL mode heads to initially
    // @Units: m
    // @Range: 1 200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RTL_ALT", 35, QuadPlane, qrtl_alt, 15),

    // @Param: RTL_MODE
    // @DisplayName: VTOL RTL mode
    // @Description: If this is set to 1 then an RTL will change to QRTL when within RTL_RADIUS meters of the RTL destination
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("RTL_MODE", 36, QuadPlane, rtl_mode, 0),

    // @Param: TILT_MASK
    // @DisplayName: Tiltrotor mask
    // @Description: This is a bitmask of motors that are tiltable in a tiltrotor (or tiltwing). The mask is in terms of the standard motor order for the frame type.
    // @User: Standard
    AP_GROUPINFO("TILT_MASK", 37, QuadPlane, tilt.tilt_mask, 0),

    // @Param: TILT_RATE_UP
    // @DisplayName: Tiltrotor upwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from forward flight to hover
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("TILT_RATE_UP", 38, QuadPlane, tilt.max_rate_up_dps, 40),

    // @Param: TILT_MAX
    // @DisplayName: Tiltrotor maximum VTOL angle
    // @Description: This is the maximum angle of the tiltable motors at which multicopter control will be enabled. Beyond this angle the plane will fly solely as a fixed wing aircraft and the motors will tilt to their maximum angle at the TILT_RATE
    // @Units: deg
    // @Increment: 1
    // @Range: 20 80
    // @User: Standard
    AP_GROUPINFO("TILT_MAX", 39, QuadPlane, tilt.max_angle_deg, 45),

    // @Param: GUIDED_MODE
    // @DisplayName: Enable VTOL in GUIDED mode
    // @Description: This enables use of VTOL in guided mode. When enabled the aircraft will switch to VTOL flight when the guided destination is reached and hover at the destination.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("GUIDED_MODE", 40, QuadPlane, guided_mode, 0),

    // 41 was used by THR_MIN

    // @Param: ESC_CAL
    // @DisplayName: ESC Calibration
    // @Description: This is used to calibrate the throttle range of the VTOL motors. Please read http://ardupilot.org/plane/docs/quadplane-esc-calibration.html before using. This parameter is automatically set back to 0 on every boot. This parameter only takes effect in QSTABILIZE mode. When set to 1 the output of all motors will come directly from the throttle stick when armed, and will be zero when disarmed. When set to 2 the output of all motors will be maximum when armed and zero when disarmed. Make sure you remove all properllers before using.
    // @Values: 0:Disabled,1:ThrottleInput,2:FullInput
    // @User: Standard
    AP_GROUPINFO("ESC_CAL", 42, QuadPlane, esc_calibration,  0),

    // @Param: VFWD_ALT
    // @DisplayName: Forward velocity alt cutoff
    // @Description: Controls altitude to disable forward velocity assist when below this relative altitude. This is useful to keep the forward velocity propeller from hitting the ground. Rangefinder height data is incorporated when available.
    // @Range: 0 10
    // @Increment: 0.25
    // @User: Standard
    AP_GROUPINFO("VFWD_ALT", 43, QuadPlane, vel_forward_alt_cutoff,  0),

    // @Param: LAND_ICE_CUT
    // @DisplayName: Cut IC engine on landing
    // @Description: This controls stopping an internal combustion engine in the final landing stage of a VTOL. This is important for aircraft where the forward thrust engine may experience prop-strike if left running during landing. This requires the engine controls are enabled using the ICE_* parameters.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("LAND_ICE_CUT", 44, QuadPlane, land_icengine_cut,  1),

    // @Param: ASSIST_ANGLE
    // @DisplayName: Quadplane assistance angle
    // @Description: This is the angular error in attitude beyond which the quadplane VTOL motors will provide stability assistance. This will only be used if Q_ASSIST_SPEED is also non-zero. Assistance will be given if the attitude is outside the normal attitude limits by at least 5 degrees and the angular error in roll or pitch is greater than this angle for at least 1 second. Set to zero to disable angle assistance.
    // @Units: deg
    // @Range: 0 90
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ASSIST_ANGLE", 45, QuadPlane, assist_angle, 30),

    // @Param: TILT_TYPE
    // @DisplayName: Tiltrotor type
    // @Description: This is the type of tiltrotor when TILT_MASK is non-zero. A continuous tiltrotor can tilt the rotors to any angle on demand. A binary tiltrotor assumes a retract style servo where the servo is either fully forward or fully up. In both cases the servo can't move faster than Q_TILT_RATE. A vectored yaw tiltrotor will use the tilt of the motors to control yaw in hover
    // @Values: 0:Continuous,1:Binary,2:VectoredYaw
    AP_GROUPINFO("TILT_TYPE", 47, QuadPlane, tilt.tilt_type, TILT_TYPE_CONTINUOUS),

    // @Param: TAILSIT_ANGLE
    // @DisplayName: Tailsitter transition angle
    // @Description: This is the angle at which tailsitter aircraft will change from VTOL control to fixed wing control.
    // @Range: 5 80
    AP_GROUPINFO("TAILSIT_ANGLE", 48, QuadPlane, tailsitter.transition_angle, 45),

    // @Param: TILT_RATE_DN
    // @DisplayName: Tiltrotor downwards tilt rate
    // @Description: This is the maximum speed at which the motor angle will change for a tiltrotor when moving from hover to forward flight. When this is zero the Q_TILT_RATE_UP value is used.
    // @Units: deg/s
    // @Increment: 1
    // @Range: 10 300
    // @User: Standard
    AP_GROUPINFO("TILT_RATE_DN", 49, QuadPlane, tilt.max_rate_down_dps, 0),

    // @Param: TAILSIT_INPUT
    // @DisplayName: Tailsitter input type
    // @Description: This controls whether stick input when hovering as a tailsitter follows the conventions for fixed wing hovering or multicopter hovering. When multicopter input is selected the roll stick will roll the aircraft in earth frame and yaw stick will yaw in earth frame. When using fixed wing input the roll and yaw sticks will control the aircraft in body frame. When corvo input mode is selected, this follows the multicopter convention when in VTOL modes, and when in FBWB and CRUISE modes follows a convention where pitch stick integrates to calculate a persistent speed demand between ARSPD_FBW_MIN and ARSPD_FBW_MAX and throttle stick is used to calculate a vertical velocity demand between FBWB_SINK_RATE and FBWB_CLIMB_RATE.
    // @Values: 0:MultiCopterInput,1:FixedWingInput,2:CorvoInput
    AP_GROUPINFO("TAILSIT_INPUT", 50, QuadPlane, tailsitter.input_type, TAILSITTER_INPUT_MULTICOPTER),

    // @Param: TAILSIT_MASK
    // @DisplayName: Tailsitter input mask
    // @Description: This controls what channels have full manual control when hovering as a tailsitter and the Q_TAILSIT_MASKCH channel in high. This can be used to teach yourself to prop-hang a 3D plane by learning one or more channels at a time.
    // @Bitmask: 0:Aileron,1:Elevator,2:Throttle,3:Rudder
    AP_GROUPINFO("TAILSIT_MASK", 51, QuadPlane, tailsitter.input_mask, 0),

    // @Param: TAILSIT_MASKCH
    // @DisplayName: Tailsitter input mask channel
    // @Description: This controls what input channel will activate the Q_TAILSIT_MASK mask. When this channel goes above 1700 then the pilot will have direct manual control of the output channels specified in Q_TAILSIT_MASK. Set to zero to disable.
    // @Values: 0:Disabled,1:Channel1,2:Channel2,3:Channel3,4:Channel4,5:Channel5,6:Channel6,7:Channel7,8:Channel8
    AP_GROUPINFO("TAILSIT_MASKCH", 52, QuadPlane, tailsitter.input_mask_chan, 0),

    // @Param: TAILSIT_VFGAIN
    // @DisplayName: Tailsitter vector thrust gain in forward flight
    // @Description: This controls the amount of vectored thrust control used in forward flight for a vectored tailsitter
    // @Range: 0 1
    // @Increment: 0.01
    AP_GROUPINFO("TAILSIT_VFGAIN", 53, QuadPlane, tailsitter.vectored_forward_gain, 0),

    // @Param: TAILSIT_VHGAIN
    // @DisplayName: Tailsitter vector thrust gain in hover
    // @Description: This controls the amount of vectored thrust control used in hover for a vectored tailsitter. For TVBS frame types it controls the amount of roll/yaw vectored thrust only. If the frame type is set to 14 (TVBS), a different algorithm is used for pitch control.
    // @Range: 0 1
    // @Increment: 0.01
    AP_GROUPINFO("TAILSIT_VHGAIN", 54, QuadPlane, tailsitter.vectored_hover_gain, 0.5),

    // @Param: TILT_YAW_ANGLE
    // @DisplayName: Tilt minimum angle for vectored yaw
    // @Description: This is the angle of the tilt servos when in VTOL mode and at minimum output. This needs to be set for Q_TILT_TYPE=3 to enable vectored control for yaw of tricopter tilt quadplanes.
    // @Range: 0 30
    AP_GROUPINFO("TILT_YAW_ANGLE", 55, QuadPlane, tilt.tilt_yaw_angle, 0),

    // @Param: TAILSIT_VHPOW
    // @DisplayName: Tailsitter vector thrust gain power
    // @Description: This controls the amount of extra pitch given to the vectored control when at high pitch errors
    // @Range: 0 4
    // @Increment: 0.1
    AP_GROUPINFO("TAILSIT_VHPOW", 56, QuadPlane, tailsitter.vectored_hover_power, 2.5),

    // @Param: MAV_TYPE
    // @DisplayName: MAVLink type identifier
    // @Description: This controls the mavlink type given in HEARTBEAT messages. For some GCS types a particular setting will be needed for correct operation.
    // @Values: 0:AUTO,1:FIXED_WING,2:QUADROTOR,3:COAXIAL,4:HELICOPTER,7:AIRSHIP,8:FREE_BALLOON,9:ROCKET,10:GROUND_ROVER,11:SURFACE_BOAT,12:SUBMARINE,16:FLAPPING_WING,17:KITE,19:VTOL_DUOROTOR,20:VTOL_QUADROTOR,21:VTOL_TILTROTOR
    AP_GROUPINFO("MAV_TYPE", 57, QuadPlane, mav_type, 0),

    // @Param: OPTIONS
    // @DisplayName: quadplane options
    // @Description: This provides a set of additional control options for quadplanes. LevelTransition means that the wings should be held level to within LEVEL_ROLL_LIMIT degrees during transition to fixed wing flight, and the vehicle will not use the vertical lift motors to climb during the transition. If AllowFWTakeoff bit is not set then fixed wing takeoff on quadplanes will instead perform a VTOL takeoff. If AllowFWLand bit is not set then fixed wing land on quadplanes will instead perform a VTOL land. If respect takeoff frame is not set the vehicle will interpret all takeoff waypoints as an altitude above the corrent position.
    // @Bitmask: 0:LevelTransition,1:AllowFWTakeoff,2:AllowFWLand,3:Respect takeoff frame types,4:Use a fixed wing approach for VTOL landings
    AP_GROUPINFO("OPTIONS", 58, QuadPlane, options, 0),

    AP_SUBGROUPEXTENSION("",59, QuadPlane, var_info2),

    AP_GROUPEND
};

// second table of user settable parameters for quadplanes, this
// allows us to go beyond the 64 parameter limit
const AP_Param::GroupInfo QuadPlane::var_info2[] = {
    // @Param: TRANS_DECEL
    // @DisplayName: Transition deceleration
    // @Description: This is deceleration rate that will be used in calculating the stopping distance when transitioning from fixed wing flight to multicopter flight.
    // @Units: m/s/s
    // @Increment: 0.1
    // @Range: 0.2 5
    // @User: Standard
    AP_GROUPINFO("TRANS_DECEL", 1, QuadPlane, transition_decel, 2.0),

    // @Group: LOIT_
    // @Path: ../libraries/AC_WPNav/AC_Loiter.cpp
    AP_SUBGROUPPTR(loiter_nav, "LOIT_",  2, QuadPlane, AC_Loiter),

    // @Param: TAILSIT_THSCMX
    // @DisplayName: Maximum control throttle scaling value
    // @Description: Maximum value of throttle scaling for tailsitter velocity scaling, reduce this value to remove low thorottle D ossilaitons 
    // @Range: 1 5
    // @User: Standard
    AP_GROUPINFO("TAILSIT_THSCMX", 3, QuadPlane, tailsitter.throttle_scale_max, 5),

    // @Param: TRIM_PITCH
    // @DisplayName: Quadplane AHRS trim pitch
    // @Description: This sets the compensation for the pitch angle trim difference between forward and vertical flight pitch, NOTE! this is relative to forward flight trim not mounting locaiton. For tailsitters this is relative to a baseline of 90 degrees.
    // @Units: deg
    // @Range: -10 +10
    // @Increment: 0.1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TRIM_PITCH", 4, QuadPlane, ahrs_trim_pitch, 0),
	
    // @Param: TVBS_DGAIN
    // @DisplayName: Gain from wing pitch acceleration to elevator
    // @Description: Sets the number of equivalent elevator servo degrees per deg/sec/sec of wing pitch acceleration. TVBS frame class only.
    // @Units: sec
    // @Range: 0.0 - 0.05
    // @Increment: 0.002
    // @User: Standard
    AP_GROUPINFO("TVBS_DGAIN", 11, QuadPlane, tailsitter.tvbs_dgain, 0.01f),

    // @Param: TVBS_DTAU
    // @DisplayName: Time constant for pitch rate derivative filter
    // @Description: Controls the time constant of the noise filter that is appleid to the pitch rate derivative used by the wing elevator pitch control loop. TVBS frame class only.
    // @Units: sec
    // @Range: 0.01 - 0.1
    // @Increment: 0.005
    // @User: Standard
    AP_GROUPINFO("TVBS_DTAU", 12, QuadPlane, tailsitter.tvbs_dtau_sec, 0.02f),

    // @Param: TVBS_E_SRMAX
    // @DisplayName: Elevator channel slew rate limit
    // @Description: Sets an upper limit on the elevator channel demand slew rate. If the amplitude of the control action produced by the pitch rate and acceleration feedback exceeds this value, then the master gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by excessive gains. The limit should be set to around 50% of the servo's specified slew rate to allow for inertia and aerodynamic load effects. Note: The master gain will not be reduced to less than 10% of the nominal value. TVBS frame class only.
    // @Units: deg/sec
    // @Range: 50 500
    // @Increment: 10.0
    // @User: Advanced
    AP_GROUPINFO("TVBS_E_SRMAX", 13, QuadPlane, tailsitter.elev_slew_rate_max_dps, 450.0f),

    // @Param: TVBS_E_SRTAU
    // @DisplayName: Elevator channel slew rate decay time constant
    // @Description: This sets the time constant used to recover the elevator channel gain after it has been reduced due to excessive servo slew rate. TVBS frame class only.
    // @Units: deg/sec
    // @Range: 0.5 5.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("TVBS_E_SRTAU", 14, QuadPlane, tailsitter.elev_slew_rate_tau_sec, 0.5f),

    // @Param: TVBS_HPF_GAIN
    // @DisplayName: Gain from rotor pitch rate to elevator
    // @Description: Sets the number of equivalent elevator servo degrees per deg/sec of rotor pitch rate. Rotor pitch rate is the sum of the tilt servo angle derivative and the body pitch rate. Used to compensate for the reaction torque generated by rotor gyroscopic moments when the rotors are tilted. The time constant for the low pass filter applied to the tilter servo derviative is controlled by TVBS_ELEV_TAU. TVBS frame class only.
    // @Units: sec
    // @Range: 0.0 1.0
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("TVBS_HPF_GAIN", 15, QuadPlane, tailsitter.tvbs_rotor_to_elev_gain, 0.0f),

    // @Param: TVBS_TILT_SLEW
    // @DisplayName: Maximum slew rate for the tilt servo
    // @Description: The demand to the tilt servo that compensates for changes in body pitch angle is reduced to respect this limit. Set to a value that leaves margin for rate feedback generated by the Q_TVBS_TILT_LAG term. TVBS frame class only.
    // @Units: msec
    // @Range: 200 - 600
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("TVBS_TILT_SLEW", 16, QuadPlane, tailsitter.tvbs_tilt_slew_lim_dps, 300.0f),

    // @Param: TVBS_ELEV_TAU
    // @DisplayName: Elevator channel tilt motor feedforward time constant
    // @Description: This sets the time constant used to filter the tilt servo derivative used by the TVBS_HPF_GAIN feed forward term. TVBS frame class only.
    // @Units: deg/sec
    // @Range: 0.01 0.1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("TVBS_ELEV_TAU", 17, QuadPlane, tailsitter.tvbs_elev_hpf_tau_sec, 0.03f),

    // 18 unassigned - used previously by TVBS_CC_GAIN

    // @Param: TVBS_ROLL_GAIN
    // @DisplayName: Gain factor applied to demanded roll angle
    // @Description: This can be used to reduce the amount of roll angle demand from the position controller. TVBS airframes have less aerodynamic drag and therefore damping of lateral movement than they do for longitudinal movement. TVBS frame class only.
    // @Units: deg/deg
    // @Range: 0.2 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("TVBS_ROLL_GAIN", 19, QuadPlane, tailsitter.tvbs_roll_gain, 1.0f),

    // @Param: TVBS_TILT_LAG
    // @DisplayName: Tilt servo lag compensation
    // @Description: This controls the amount of motor tilt time lag that is compensated for. Increasing it increases the amount of pitch damping contributed by the motor tilt, but also makes the tilt servos work harder and can cause limit cycling and loss of pitch control if a large amplutude disturbance occurs. TVBS frame class only.
    // @Units: msec
    // @Range: 0 100
    // @Increment: 5
    // @User: Standard
    AP_GROUPINFO("TVBS_TILT_LAG", 20, QuadPlane, tailsitter.tvbs_tilt_lag_ms, 50),

    // @Param: TVBS_ANGMAX
    // @DisplayName: Thrust line elevation upper limit
    // @Description: Set this to the most positive number of degrees of thrust line rotation achieved when the tilt servo is at the travel limit as set by the servos max and min PWM parameters. Zero is defined by the the thrust line pointing along the X body axis. A positive rotation is a nose up or a RH rotation about the Y body axis. The servo mechanism and parameters must be adjusted so that the zero rotation is achieved when the servo is at the trim PWM value. TVBS frame class only.
    // @Units: deg
    // @Range: 15 120
    // @Increment: 5
    // @User: Standard
    AP_GROUPINFO("TVBS_ANGMAX", 21, QuadPlane, tailsitter.tvbs_ang_max_deg, 90),

    // @Param: TVBS_ANGMIN
    // @DisplayName: Thrust line elevation lower limit
    // @Description: Set this to the most negative number of degrees of thrust line rotation achieved when the tilt servo is at the travel limit as set by the servos max and min PWM parameters. Zero is defined by the the thrust line pointing along the X body axis. A negtive rotation is nose down or a LH rotation about the Y body axis. The servo mechanism and parameters must be adjusted so that the zero rotation is achieved when the servo is at the trim PWM value. TVBS frame class only.
    // @Units: deg
    // @Range: -120 -15
    // @Increment: 5
    // @User: Standard
    AP_GROUPINFO("TVBS_ANGMIN", 22, QuadPlane, tailsitter.tvbs_ang_min_deg, -90),

    // @Param: TVBS_PGAIN
    // @DisplayName: Gain from pitch rate to elevator
    // @Description: Sets the number of equivalent elevator servo degrees per deg/sec of pitch rate used to damp body pitch motion during VTOL operation. TVBS frame class only.
    // @Units: sec
    // @Range: 0.0 - 0.5
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("TVBS_PGAIN", 23, QuadPlane, tailsitter.tvbs_rate_gain, 0.15f),

    // @Param: TVBS_SLEW_LIM
    // @DisplayName: Rotor pitch rate limit.
    // @Description: Sets the maximum allowed rate of change of the demanded rotor pitch angle. TVBS frame class only.
    // @Units: deg/sec
    // @Range: 60 - 240
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("TVBS_SLEW_LIM", 24, QuadPlane, tailsitter.tvbs_slew_lim_dps, 120),

    // @Param: TVBS_SLEW_TAU
    // @DisplayName: Rotor pitch time constant
    // @Description: Sets the time constant of the low pass filter that is applied to the demanded rotor pitch angle. TVBS frame class only.
    // @Units: msec
    // @Range: 0 - 200
    // @Increment: 10
    // @User: Advanced
    AP_GROUPINFO("TVBS_SLEW_TAU", 25, QuadPlane, tailsitter.tvbs_slew_tau_msec, 100),

    // @Param: TVBS_BT_PITCH
    // @DisplayName: Back transition initial pullup pitch angle
    // @Description: During the first part of back transition, the vehicle pulls up to this pitch angle and closes the throttle to decelerate prior to switching to VTOL control ans tilting rotors up. TVBS frame class only.
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TVBS_BT_PITCH", 26, QuadPlane, tailsitter.tvbs_bt_pitch, 10),

    // 27 unassigned - used previously by TVBS_LPF_GAIN

    // @Param: TVBS_BT_TIME
    // @DisplayName: Maximum time in msec to transition back from FW to RW operation
    // @Description: Sets the maximum number of milliseconds required to compete the FW to RW transition. Transition will complete sooner if vehicle forward speed is less than 1 m/s or the wing has dropped to within 10 deg of vertical
    // @Units: msec
    // @Range: 0 6000
    // @Increment: 100
    // @User: Standard
    AP_GROUPINFO("TVBS_BT_TIME", 28, QuadPlane, tailsitter.tvbs_bt_time_msec, 5000),

    // @Param: TVBS_ELEV_TRIM
    // @DisplayName: Elevator trim percentage used during RW operation
    // @Description: Use this to specify the elevator as a percentage of full throw used to trim the wing during hover so that it produces zero normal force. Positive values produce a positive pitching moment.
    // @Units: %
    // @Range: -100 +100
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TVBS_ELEV_TRIM", 29, QuadPlane, tailsitter.tvbs_elev_trim_pcnt, 0),

    // @Param: TVBS_AIL_GF
    // @DisplayName: Aileron control surface gain reduction applied during forward flight
    // @Description: Specify the maximum gain factor percentage reduction applied to the aileron deflection when the wing reaches a horizontal flying orientation. Increase only if high frequency roll limit cycling is encountered during high speed forward or reverse flight. TVBS frame class only.
    // @Range: 0 90
    // @Increment: 5
    // @User: Standard
    AP_GROUPINFO("TVBS_AIL_GF", 30, QuadPlane, tailsitter.tvbs_ail_gf, 0),

    // @Param: TVBS_ELE_GF
    // @DisplayName: Aileron control surface gain reduction applied during forward flight
    // @Description: Specify the maximum gain factor percentage reduction applied to the elevator pitch damping when the wing reaches a horizontal flying orientation. Increase only if high frequency pitch limit cycling is encountered during high speed forward or reverse flight. TVBS frame class only.
    // @Range: 0 90
    // @Increment: 5
    // @User: Standard
    AP_GROUPINFO("TVBS_ELEV_GF", 31, QuadPlane, tailsitter.tvbs_elev_gf, 0),

    // @Param: TVBS_AR_TUNE
    // @DisplayName: Activate tuning mode for attitude recovery
    // @Description: Set to 85 to put pitch controller into auto recovery mode but retain normal throttle control. This enables the Q_A_* pitch angle and rate controller gains to be tuned. TVBS frame class only.
    // @Values: 0:Disabled,85:Enabled
    // @User: Standard
    AP_GROUPINFO("TVBS_AR_TUNE", 32, QuadPlane, tailsitter.tvbs_ar_tune, 0),

    // @Param: TVBS_AR_GAIN
    // @DisplayName: Gain factor applied during attitude recovery
    // @Description: All thrust vectoring gains will be multiplied by this value during attitude recovery. TVBS frame class only.
    // @Range: 0.1 1.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("TVBS_AR_GAIN", 33, QuadPlane, tailsitter.tvbs_ar_gain, 0.25),

    // @Param: TVBS_FW_A_GAIN
    // @DisplayName: Gain from aileron to thrust angle used during fixed wing flight modes
    // @Description: Specifies the gain from demanded aileron to demanded thrust angle during fixed wing flight modes. Only applied when the magnitude of equivalent aileron servo deflection exceeds TVBS_FW_A_DZ. TVBS frame class only.
    // @Range: 0.0 1.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("TVBS_FW_A_GAIN", 34, QuadPlane, tailsitter.tvbs_fw_ail_fwd_gain, 0.0),

    // @Param: TVBS_FW_A_DZ
    // @DisplayName: Aileron to thrust angle dead zone used during fixed wing flight modes
    // @Description: Specifies the number of degrees of equivalent aileron deflection required before thrust vectoring is added to augment control authority in forward flight modes. The gain from aileron to thrust angle controlled by TVBS_FW_A_GAIN. TVBS frame class only.
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TVBS_FW_A_DZ", 35, QuadPlane, tailsitter.tvbs_fw_ail_deadband_deg, 0),

    // @Param: TVBS_FW_E_GAIN
    // @DisplayName: Gain from elevator to thrust angle used during fixed wing flight modes
    // @Description: Specifies the gain from demanded elevator to demanded thrust angle during fixed wing flight modes. Only applied when the magnitude of equivalent elevator servo deflection exceeds TVBS_FW_E_DZ. TVBS frame class only.
    // @Range: 0.0 1.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("TVBS_FW_E_GAIN", 36, QuadPlane, tailsitter.tvbs_fw_elev_fwd_gain, 0.0),

    // @Param: TVBS_FW_E_DZ
    // @DisplayName: Elevator to thrust angle dead zone used during fixed wing flight modes
    // @Description: Specifies the number of degrees of equivalent elevator deflection required before thrust vectoring is added to augment control authority in forward flight modes. The gain from elevator to thrust angle controlled by TVBS_FW_E_GAIN. TVBS frame class only.
    // @Units: deg
    // @Range: 0 45
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TVBS_FW_E_DZ", 37, QuadPlane, tailsitter.tvbs_fw_elev_deadband_deg, 0),

    // @Param: TVBS_TO_SCALER
    // @DisplayName: Max scale factor applied to wind drift integrator during initial takeoff.
    // @Description: Controls how fast the wind drift is learned during the first 10 metres of climb
    // @Range: 1.0 10.0
    // @Increment: 0.5
    // @User: Standard
    AP_GROUPINFO("TVBS_TO_SCALER", 38, QuadPlane, tailsitter.tvbs_to_scaler, 5.0f),

    // @Param: TVBS_WPE_GAIN
    // @DisplayName: Gain from wing pitch angle error to elevator.
    // @Description: Used in high rotor tilt situations to allow the wing to re-trim to follow the rotor tilt, reduce wing drag and allow a larger forward speed in VTOL modes.
    // @Range: 0.0 2.0
    // @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("TVBS_WPE_GAIN", 39, QuadPlane, tailsitter.tvbs_wpe_gain, 0.0f),

    // @Param: TVBS_LND_CONE
    // @DisplayName: Elevation angle of the landing recovery cone.
    // @Description: The vehicle is not allowed to descend if outside an inverted cone with a truncated vertex located on the landing point. This parameter specifies the elevation angle of the cone above the horizon.
    // @Range: 0 80
    // @Increment: 1
    // @Units: deg
    // @User: Standard
    AP_GROUPINFO("TVBS_LND_CONE", 40, QuadPlane, tailsitter.tvbs_land_cone_elev, 60),

    // @Param: TVBS_LND_RAD
    // @DisplayName: Vertex radius of the landing recovery cone.
    // @Description: The vehicle is not allowed to descend if outside an inverted cone with a truncated vertex located on the landing point. This parameter specifies the radius of the cone vertex.
    // @Range: 0 10
    // @Increment: 1
    // @Units: m
    // @User: Standard
    AP_GROUPINFO("TVBS_LND_RAD", 41, QuadPlane, tailsitter.tvbs_land_cone_radius, 5),

    // @Param: TVBS_YAW_GAIN
    // @DisplayName: Gain from payload yaw offset to vehicle demanded yaw rate.
    // @Description: Used during VTOL operation where the operator is yawing the camera. Enables the vehicle to yaw to enable full 360deg camera pans when the mount do has limited movement. Set to zero to disable.
    // @Range: 0.0 3.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("TVBS_YAW_GAIN", 42, QuadPlane, tailsitter.tvbs_yaw_gain, 1.5f),

    // @Param: TVBS_LAT_GMAX
    // @DisplayName: Maximum lateral g allowed during yaw to payload
    // @Description: Used during VTOL operation where the operator is yawing the camera and we want the vehicle yaw to follow the paylad yaw. This paameter sets the maximum allowed lateral acceleration before the yaw to follow payload is ignored.
    // @Range: 0.05 0.3
    // @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("TVBS_LAT_GMAX", 43, QuadPlane, tailsitter.tvbs_lat_gmax, 0.2f),

    // @Param: TVBS_JMP_ALT
    // @DisplayName: Takeoff jump altitude
    // @Description: Sets the altitude that the vehicle will jump to when taking off in QLOITER when Q_TAILSIT_INPUT is set to 2 (using corvo X hand controller).
    // @Units: m
    // @Range: 1.0 5.0
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("TVBS_JMP_ALT", 44, QuadPlane, tailsitter.tvbs_jmp_alt, 2.5),

    // @Param: TVBS_JMP_RAD
    // @DisplayName: Takeoff jump radius
    // @Description: Controls the radius of the takeoff/landing zone used when flying in QLOITER when Q_TAILSIT_INPUT is set to 2 (using corvo X hand controller). When within this radfius, the vehicle must climb above Q_RTL_ALT before a forward flight transition is allowed and when descending will automatically reduced sink rate when close to the pre-takeoff recorded altitude to prevent hard landings.
    // @Units: m
    // @Range: 0 127
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("TVBS_JMP_RAD", 45, QuadPlane, tailsitter.tvbs_jmp_radius, 50),
    AP_GROUPEND
};

struct defaults_struct {
    const char *name;
    float value;
};

/*
  defaults for all quadplanes
 */
static const struct defaults_struct defaults_table[] = {
    { "Q_A_RAT_RLL_P",    0.25 },
    { "Q_A_RAT_RLL_I",    0.25 },
    { "Q_A_RAT_RLL_FILT", 10.0 },
    { "Q_A_RAT_PIT_P",    0.25 },
    { "Q_A_RAT_PIT_I",    0.25 },
    { "Q_A_RAT_PIT_FILT", 10.0 },
    { "Q_M_SPOOL_TIME",   0.25 },
    { "Q_LOIT_ANG_MAX",   15.0 },
    { "Q_LOIT_ACC_MAX",   250.0 },
    { "Q_LOIT_BRK_ACCEL", 50.0 },
    { "Q_LOIT_BRK_JERK",  250 },
    { "Q_LOIT_SPEED",     500 },
};

/*
  extra defaults for tailsitters
 */
static const struct defaults_struct defaults_table_tailsitter[] = {
    { "KFF_RDDRMIX",       0.02 },
    { "Q_A_RAT_PIT_FF",    0.2 },
    { "Q_A_RAT_YAW_FF",    0.2 },
    { "Q_A_RAT_YAW_I",    0.18 },
    { "LIM_PITCH_MAX",    3000 },
    { "LIM_PITCH_MIN",    -3000 },
    { "MIXING_GAIN",      1.0 },
    { "RUDD_DT_GAIN",      10 },
    { "Q_TRANSITION_MS",   2000 },
};

/*
  conversion table for quadplane parameters
 */
const AP_Param::ConversionInfo q_conversion_table[] = {
    { Parameters::k_param_quadplane, 4044, AP_PARAM_FLOAT, "Q_P_POSZ_P" },     //  Q_PZ_P
    { Parameters::k_param_quadplane, 4045, AP_PARAM_FLOAT, "Q_P_POSXY_P"},     //  Q_PXY_P
    { Parameters::k_param_quadplane, 4046, AP_PARAM_FLOAT, "Q_P_VELXY_P"},     //  Q_VXY_P
    { Parameters::k_param_quadplane, 78,   AP_PARAM_FLOAT, "Q_P_VELXY_I"},     //  Q_VXY_I
    { Parameters::k_param_quadplane, 142,  AP_PARAM_FLOAT, "Q_P_VELXY_IMAX"},  //  Q_VXY_IMAX
    { Parameters::k_param_quadplane, 206,  AP_PARAM_FLOAT, "Q_P_VELXY_FILT"},  //  Q_VXY_FILT_HZ
    { Parameters::k_param_quadplane, 4047, AP_PARAM_FLOAT, "Q_P_VELZ_P"},      //  Q_VZ_P
    { Parameters::k_param_quadplane, 4048, AP_PARAM_FLOAT, "Q_P_ACCZ_P"},      //  Q_AZ_P
    { Parameters::k_param_quadplane, 80,   AP_PARAM_FLOAT, "Q_P_ACCZ_I"},      //  Q_AZ_I
    { Parameters::k_param_quadplane, 144,  AP_PARAM_FLOAT, "Q_P_ACCZ_D"},      //  Q_AZ_D
    { Parameters::k_param_quadplane, 336,  AP_PARAM_FLOAT, "Q_P_ACCZ_IMAX"},   //  Q_AZ_IMAX
    { Parameters::k_param_quadplane, 400,  AP_PARAM_FLOAT, "Q_P_ACCZ_FILT"},   //  Q_AZ_FILT
    { Parameters::k_param_quadplane, 464,  AP_PARAM_FLOAT, "Q_P_ACCZ_FF"},     //  Q_AZ_FF
    { Parameters::k_param_quadplane, 276,  AP_PARAM_FLOAT, "Q_LOIT_SPEED"},    //  Q_WP_LOIT_SPEED
    { Parameters::k_param_quadplane, 468,  AP_PARAM_FLOAT, "Q_LOIT_BRK_JERK" },//  Q_WP_LOIT_JERK
    { Parameters::k_param_quadplane, 532,  AP_PARAM_FLOAT, "Q_LOIT_ACC_MAX" }, //  Q_WP_LOIT_MAXA
    { Parameters::k_param_quadplane, 596,  AP_PARAM_FLOAT, "Q_LOIT_BRK_ACCEL" },// Q_WP_LOIT_MINA
};


QuadPlane::QuadPlane(AP_AHRS_NavEKF &_ahrs) :
    ahrs(_ahrs)
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::setup_object_defaults(this, var_info2);
}


// setup default motors for the frame class
void QuadPlane::setup_default_channels(uint8_t num_motors)
{
    for (uint8_t i=0; i<num_motors; i++) {
        SRV_Channels::set_aux_channel_default(SRV_Channels::get_motor_function(i), CH_5+i);
    }
}
    

bool QuadPlane::setup(void)
{
    if (initialised) {
        return true;
    }
    if (!enable || hal.util->get_soft_armed()) {
        return false;
    }
    float loop_delta_t = 1.0 / plane.scheduler.get_loop_rate_hz();

    enum AP_Motors::motor_frame_class motor_class;
    enum Rotation rotation = ROTATION_NONE;
        
    /*
      cope with upgrade from old AP_Motors values for frame_class
     */
    AP_Int8 old_class;
    const AP_Param::ConversionInfo cinfo { Parameters::k_param_quadplane, FRAME_CLASS_OLD_IDX, AP_PARAM_INT8, nullptr };
    if (AP_Param::find_old_parameter(&cinfo, &old_class) && !frame_class.load()) {
        uint8_t new_value = 0;
        // map from old values to new values
        switch (old_class.get()) {
        case 0:
            new_value = AP_Motors::MOTOR_FRAME_QUAD;
            break;
        case 1:
            new_value = AP_Motors::MOTOR_FRAME_HEXA;
            break;
        case 2:
            new_value = AP_Motors::MOTOR_FRAME_OCTA;
            break;
        case 3:
            new_value = AP_Motors::MOTOR_FRAME_OCTAQUAD;
            break;
        case 4:
            new_value = AP_Motors::MOTOR_FRAME_Y6;
            break;
        }
        frame_class.set_and_save(new_value);
    }
    
    if (hal.util->available_memory() <
        4096 + sizeof(*motors) + sizeof(*attitude_control) + sizeof(*pos_control) + sizeof(*wp_nav) + sizeof(*ahrs_view) + sizeof(*loiter_nav)) {
        gcs().send_text(MAV_SEVERITY_INFO, "Not enough memory for quadplane");
        goto failed;
    }

    /*
      dynamically allocate the key objects for quadplane. This ensures
      that the objects don't affect the vehicle unless enabled and
      also saves memory when not in use
     */
    motor_class = (enum AP_Motors::motor_frame_class)frame_class.get();
    switch (motor_class) {
    case AP_Motors::MOTOR_FRAME_QUAD:
        setup_default_channels(4);
        break;
    case AP_Motors::MOTOR_FRAME_HEXA:
        setup_default_channels(6);
        break;
    case AP_Motors::MOTOR_FRAME_OCTA:
    case AP_Motors::MOTOR_FRAME_OCTAQUAD:
        setup_default_channels(8);
        break;
    case AP_Motors::MOTOR_FRAME_Y6:
        setup_default_channels(7);
        break;
    case AP_Motors::MOTOR_FRAME_TRI:
        SRV_Channels::set_default_function(CH_5, SRV_Channel::k_motor1);
        SRV_Channels::set_default_function(CH_6, SRV_Channel::k_motor2);
        SRV_Channels::set_default_function(CH_8, SRV_Channel::k_motor4);
        SRV_Channels::set_default_function(CH_11, SRV_Channel::k_motor7);
        AP_Param::set_frame_type_flags(AP_PARAM_FRAME_TRICOPTER);
        break;
    case AP_Motors::MOTOR_FRAME_TAILSITTER:
        break;
    case AP_Motors::MOTOR_FRAME_TVBS:
        _elev_slew_rate_filter.reset(0.0f);
        _elev_slew_rate_filter.set_cutoff_frequency(5.0f);
        break;
    default:
        hal.console->printf("Unknown frame class %u - using QUAD\n", (unsigned)frame_class.get());
        frame_class.set(AP_Motors::MOTOR_FRAME_QUAD);
        setup_default_channels(4);
        break;
    }

    switch (motor_class) {
    case AP_Motors::MOTOR_FRAME_TRI:
        motors = new AP_MotorsTri(plane.scheduler.get_loop_rate_hz(), rc_speed);
        motors_var_info = AP_MotorsTri::var_info;
        break;
    case AP_Motors::MOTOR_FRAME_TAILSITTER:
    case AP_Motors::MOTOR_FRAME_TVBS:
        motors = new AP_MotorsTailsitter(plane.scheduler.get_loop_rate_hz(), rc_speed);
        motors_var_info = AP_MotorsTailsitter::var_info;
        rotation = ROTATION_PITCH_90;
        break;
    default:
        motors = new AP_MotorsMatrix(plane.scheduler.get_loop_rate_hz(), rc_speed);
        motors_var_info = AP_MotorsMatrix::var_info;
        break;
    }
    const static char *strUnableToAllocate = "Unable to allocate";
    if (!motors) {
        hal.console->printf("%s motors\n", strUnableToAllocate);
        goto failed;
    }

    AP_Param::load_object_from_eeprom(motors, motors_var_info);

    // create the attitude view used by the VTOL code
    ahrs_view = ahrs.create_view(rotation, ahrs_trim_pitch);
    if (ahrs_view == nullptr) {
        goto failed;
    }

    attitude_control = new AC_AttitudeControl_Multi(*ahrs_view, aparm, *motors, loop_delta_t);
    if (!attitude_control) {
        hal.console->printf("%s attitude_control\n", strUnableToAllocate);
        goto failed;
    }
    AP_Param::load_object_from_eeprom(attitude_control, attitude_control->var_info);
    pos_control = new AC_PosControl(*ahrs_view, ahrs, inertial_nav, *motors, *attitude_control);
    if (!pos_control) {
        hal.console->printf("%s pos_control\n", strUnableToAllocate);
        goto failed;
    }
    AP_Param::load_object_from_eeprom(pos_control, pos_control->var_info);
    wp_nav = new AC_WPNav(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (!wp_nav) {
        hal.console->printf("%s wp_nav\n", strUnableToAllocate);
        goto failed;
    }
    AP_Param::load_object_from_eeprom(wp_nav, wp_nav->var_info);

    loiter_nav = new AC_Loiter(inertial_nav, *ahrs_view, *pos_control, *attitude_control);
    if (!loiter_nav) {
        hal.console->printf("%s loiter_nav\n", strUnableToAllocate);
        goto failed;
    }
    AP_Param::load_object_from_eeprom(loiter_nav, loiter_nav->var_info);

    motors->init((AP_Motors::motor_frame_class)frame_class.get(), (AP_Motors::motor_frame_type)frame_type.get());
    motors->set_throttle_range(thr_min_pwm, thr_max_pwm);
    motors->set_update_rate(rc_speed);
    motors->set_interlock(true);
    pos_control->set_dt(loop_delta_t);
    attitude_control->parameter_sanity_check();

    // setup the trim of any motors used by AP_Motors so px4io
    // failsafe will disable motors
    for (uint8_t i=0; i<8; i++) {
        SRV_Channel::Aux_servo_function_t func = SRV_Channels::get_motor_function(i);
        SRV_Channels::set_failsafe_pwm(func, thr_min_pwm);
    }

#if HAVE_PX4_MIXER
    // redo failsafe mixing on px4
    plane.setup_failsafe_mixing();
#endif
    
    transition_state = TRANSITION_DONE;

    if (tilt.tilt_mask != 0 && tilt.tilt_type == TILT_TYPE_VECTORED_YAW) {
        // setup tilt servos for vectored yaw
        SRV_Channels::set_range(SRV_Channel::k_tiltMotorLeft,  1000);
        SRV_Channels::set_range(SRV_Channel::k_tiltMotorRight, 1000);
    }

    
    setup_defaults();

    AP_Param::convert_old_parameters(&q_conversion_table[0], ARRAY_SIZE(q_conversion_table));
    
    gcs().send_text(MAV_SEVERITY_INFO, "QuadPlane initialised");
    initialised = true;
    return true;
    
failed:
    initialised = false;
    enable.set(0);
    gcs().send_text(MAV_SEVERITY_INFO, "QuadPlane setup failed");
    return false;
}

/*
  setup default parameters from a defaults_struct table
 */
void QuadPlane::setup_defaults_table(const struct defaults_struct *table, uint8_t count)
{
    for (uint8_t i=0; i<count; i++) {
        if (!AP_Param::set_default_by_name(table[i].name, table[i].value)) {
            gcs().send_text(MAV_SEVERITY_INFO, "QuadPlane setup failure for %s",
                                             table[i].name);
            AP_HAL::panic("quadplane bad default %s", table[i].name);
        }
    }
}

/*
  setup default parameters from defaults_table
 */
void QuadPlane::setup_defaults(void)
{
    setup_defaults_table(defaults_table, ARRAY_SIZE(defaults_table));

    enum AP_Motors::motor_frame_class motor_class;
    motor_class = (enum AP_Motors::motor_frame_class)frame_class.get();
    if ((motor_class == AP_Motors::MOTOR_FRAME_TAILSITTER) || (motor_class == AP_Motors::MOTOR_FRAME_TVBS)) {
        setup_defaults_table(defaults_table_tailsitter, ARRAY_SIZE(defaults_table_tailsitter));
    }
    
    // reset ESC calibration
    if (esc_calibration != 0) {
        esc_calibration.set_and_save(0);
    }
}

// run ESC calibration
void QuadPlane::run_esc_calibration(void)
{
    if (!motors->armed()) {
        motors->set_throttle_passthrough_for_esc_calibration(0);
        AP_Notify::flags.esc_calibration = false;
        return;
    }
    if (!AP_Notify::flags.esc_calibration) {
        gcs().send_text(MAV_SEVERITY_INFO, "Starting ESC calibration");
    }
    AP_Notify::flags.esc_calibration = true;
    switch (esc_calibration) {
    case 1:
        // throttle based calibration
        motors->set_throttle_passthrough_for_esc_calibration(plane.get_throttle_input() * 0.01f);
        break;
    case 2:
        // full range calibration
        motors->set_throttle_passthrough_for_esc_calibration(1);
        break;
    }
}


// init quadplane stabilize mode 
void QuadPlane::init_stabilize(void)
{
    throttle_wait = false;
}


/*
  ask the multicopter attitude control to match the roll and pitch rates being demanded by the
  fixed wing controller if not in a pure VTOL mode
 */
void QuadPlane::multicopter_attitude_rate_update(float yaw_rate_cds)
{
    check_attitude_relax();

    if (in_vtol_mode() || is_tailsitter()) {
        // use euler angle attitude control
        if (control_loss_declared) {
             attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0,0,0.0f);
        } else {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                      ahrs_view->pitch_sensor,
                                                                      yaw_rate_cds);
        }
    } else {
        // use the fixed wing desired rates
        float roll_rate = plane.rollController.get_pid_info().desired;
        float pitch_rate = plane.pitchController.get_pid_info().desired;
        attitude_control->input_rate_bf_roll_pitch_yaw_2(roll_rate*100.0f, pitch_rate*100.0f, yaw_rate_cds);
    }
}

// hold in stabilize with given throttle
void QuadPlane::hold_stabilize(float throttle_in)
{    
    // call attitude controller
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds());

    if (throttle_in <= 0) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        if (is_tailsitter()) {
            // always stabilize with tailsitters so we can do belly takeoffs
            attitude_control->set_throttle_out(0, true, 0);
        } else {
            attitude_control->set_throttle_out_unstabilized(0, true, 0);
        }
    } else {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        attitude_control->set_throttle_out(throttle_in, true, 0);
    }
}

// quadplane stabilize mode
void QuadPlane::control_stabilize(void)
{
    // special check for ESC calibration in QSTABILIZE
    if (esc_calibration != 0) {
        run_esc_calibration();
        return;
    }

    // normal QSTABILIZE mode
    float pilot_throttle_scaled = plane.get_throttle_input() / 100.0f;
    hold_stabilize(pilot_throttle_scaled);

}

// run the multicopter Z controller
void QuadPlane::run_z_controller(void)
{
    const uint32_t now = AP_HAL::millis();
    if (now - last_pidz_active_ms > 2000) {
        // set alt target to current height on transition. This
        // starts the Z controller off with the right values
        gcs().send_text(MAV_SEVERITY_INFO, "Reset alt target to %.1f", (double)inertial_nav.get_altitude() / 100);
        set_alt_target_current();
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());

        // initialize vertical speeds and leash lengths
        pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
        pos_control->set_max_accel_z(pilot_accel_z);
        
        // it has been two seconds since we last ran the Z
        // controller. We need to assume the integrator may be way off
        // the base throttle we start at is the current throttle we are using
        float base_throttle = constrain_float(motors->get_throttle() - motors->get_throttle_hover(), -1, 1) * 1000;
        pos_control->get_accel_z_pid().set_integrator(base_throttle);

        // run the controller and record the time
        pos_control->update_z_controller();
        last_pidz_active_ms = now;

    } else {
        // run the controller and record the time
        pos_control->update_z_controller();
        last_pidz_active_ms = now;

    }

}

/*
  check if we should relax the attitude controllers

  We relax them whenever we will be using them after a period of
  inactivity
 */
void QuadPlane::check_attitude_relax(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_att_control_ms > 100) {
        attitude_control->relax_attitude_controllers();
    }
    last_att_control_ms = now;
}

// init quadplane hover mode 
void QuadPlane::init_hover(void)
{
    // initialize vertical speeds and leash lengths
    pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_max_accel_z(pilot_accel_z);

    // initialise position and desired velocity
    set_alt_target_current();
    pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());

    init_throttle_wait();
}

/*
  check for an EKF yaw reset
 */
void QuadPlane::check_yaw_reset(void)
{
    float yaw_angle_change_rad = 0.0f;
    uint32_t new_ekfYawReset_ms = ahrs.getLastYawResetAngle(yaw_angle_change_rad);
    if (new_ekfYawReset_ms != ekfYawReset_ms) {
        attitude_control->inertial_frame_reset();
        ekfYawReset_ms = new_ekfYawReset_ms;
        gcs().send_text(MAV_SEVERITY_INFO, "EKF yaw reset %.2f", (double)degrees(yaw_angle_change_rad));
    }
}

/*
  hold hover with target climb rate
 */
void QuadPlane::hold_hover(float target_climb_rate)
{
    // motors use full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_max_accel_z(pilot_accel_z);

    // call attitude controller
    multicopter_attitude_rate_update(get_desired_yaw_rate_cds());

    // call position controller
    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, plane.G_Dt, false);
    run_z_controller();
}

/*
  control QHOVER mode
 */
void QuadPlane::control_hover(void)
{
    if (throttle_wait) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0, true, 0);
        pos_control->relax_alt_hold_controllers(0);
    } else {
        hold_hover(get_pilot_desired_climb_rate_cms());
    }
}

void QuadPlane::init_loiter(void)
{
    // initialise loiter
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_max_accel_z(pilot_accel_z);

    // initialise position and desired velocity
    set_alt_target_current();
    pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());

    init_throttle_wait();

    // remember initial pitch
    loiter_initial_pitch_cd = MAX(plane.ahrs.pitch_sensor, 0);

    // prevent re-init of target position
    last_loiter_ms = AP_HAL::millis();
}

// helper for is_flying()
bool QuadPlane::is_flying(void)
{
    if (!available()) {
        return false;
    }
    if (plane.control_mode == GUIDED && guided_takeoff) {
        return true;
    }
    if (motors->get_throttle() > 0.01f && !motors->limit.throttle_lower) {
        return true;
    }
    if (in_tailsitter_vtol_transition()) {
        return true;
    }
    return false;
}

// crude landing detector to prevent tipover
bool QuadPlane::should_relax(void)
{
    const uint32_t tnow = millis();

    bool motor_at_lower_limit = motors->limit.throttle_lower && attitude_control->is_throttle_mix_min();
    if (motors->get_throttle() < 0.01f) {
        motor_at_lower_limit = true;
    }

    if (!motor_at_lower_limit) {
        landing_detect.lower_limit_start_ms = 0;
        return false;
    } else if (landing_detect.lower_limit_start_ms == 0) {
        landing_detect.lower_limit_start_ms = tnow;
    }

    return (tnow - landing_detect.lower_limit_start_ms) > 1000;
}

// see if we are flying in vtol
bool QuadPlane::is_flying_vtol(void) const
{
    if (!available()) {
        return false;
    }
    if (motors->get_throttle() > 0.01f) {
        // if we are demanding more than 1% throttle then don't consider aircraft landed
        return true;
    }
    if (plane.control_mode == GUIDED && guided_takeoff) {
        return true;
    }
    if (plane.control_mode == QSTABILIZE || plane.control_mode == QHOVER || plane.control_mode == QLOITER) {
        // in manual flight modes only consider aircraft landed when pilot demanded throttle is zero
        return plane.get_throttle_input() > 0;
    }
    if (in_vtol_mode() && millis() - landing_detect.lower_limit_start_ms > 5000) {
        // use landing detector
        return true;
    }
    return false;
}

/*
  smooth out descent rate for landing to prevent a jerk as we get to
  land_final_alt. 
 */
float QuadPlane::landing_descent_rate_cms(float height_above_ground) const
{
    float ret = linear_interpolate(land_speed_cms, wp_nav->get_speed_down(),
                                   height_above_ground,
                                   land_final_alt, land_final_alt+6);
    return ret;
}


// run quadplane loiter controller
void QuadPlane::control_loiter()
{
    if (throttle_wait) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        attitude_control->set_throttle_out_unstabilized(0, true, 0);
        pos_control->relax_alt_hold_controllers(0);
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
        return;
    }

    check_attitude_relax();

    if (should_relax()) {
        loiter_nav->soften_for_landing();
    }

    const uint32_t now = AP_HAL::millis();
    if (now - last_loiter_ms > 500) {
        loiter_nav->clear_pilot_desired_acceleration();
        loiter_nav->init_target();
    }
    last_loiter_ms = now;

    // motors use full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_max_accel_z(pilot_accel_z);

    // process pilot's roll and pitch input
    if (plane.vtolCameraControlMode) {
        // don't move vehicle because we are using sticks to control camera
        loiter_nav->set_pilot_desired_acceleration(0, 0, plane.G_Dt);
    } else {
        loiter_nav->set_pilot_desired_acceleration(plane.channel_roll->get_control_in(),
                                                   plane.channel_pitch->get_control_in(),
                                                   plane.G_Dt);
    }

    // run loiter controller
    loiter_nav->update();

    // nav roll and pitch are controlled by the loiter controller
    if (!reverse_transition_active) {
        plane.nav_roll_cd = loiter_nav->get_roll();
        plane.nav_pitch_cd = loiter_nav->get_pitch();
    } else {
        plane.nav_roll_cd = 0;
        plane.nav_pitch_cd = 0;
    }

    if (now - last_pidz_init_ms < (uint32_t)transition_time_ms*2 && !is_tailsitter()) {
        // we limit pitch during initial transition
        float pitch_limit_cd = linear_interpolate(loiter_initial_pitch_cd, aparm.angle_max,
                                                  now,
                                                  last_pidz_init_ms, last_pidz_init_ms+transition_time_ms*2);
        if (plane.nav_pitch_cd > pitch_limit_cd) {
            plane.nav_pitch_cd = pitch_limit_cd;
            pos_control->set_limit_accel_xy();            
        }
    }
    
    
    // call attitude controller with conservative smoothing gain of 4.0f
    if (control_loss_declared) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0,0,0.0f);
    } else {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                  ahrs_view->pitch_sensor,
                                                                  get_desired_yaw_rate_cds());
    }

    // height control
    if (plane.control_mode == QLAND) {
        if (_height_above_ground_m < land_final_alt && poscontrol.state < QPOS_LAND_FINAL) {
            poscontrol.state = QPOS_LAND_FINAL;
            // cut IC engine if enabled
            if (land_icengine_cut != 0) {
                plane.g2.ice_control.engine_control(0, 0, 0);
            }
        }
        float descent_rate = (poscontrol.state == QPOS_LAND_FINAL)? land_speed_cms:landing_descent_rate_cms(_height_above_ground_m);
        pos_control->set_alt_target_from_climb_rate(-descent_rate, plane.G_Dt, true);
        check_land_complete();
    } else if (plane.control_mode == GUIDED && guided_takeoff) {
        pos_control->set_alt_target_from_climb_rate_ff(0, plane.G_Dt, false);
    } else {
        if (_doing_takeoff_jump) {
            // during takeoff jump, climb at max climb rate allowed during pilot controlled operation
            pos_control->add_takeoff_climb_rate(pilot_velocity_z_max, plane.G_Dt);
        } else {
            // update altitude target using pilot demanded climb rate
            pos_control->set_alt_target_from_climb_rate_ff(get_pilot_desired_climb_rate_cms(), plane.G_Dt, false);
        }
    }

    // run height controller calculations
    run_z_controller();
}

/*
  get pilot input yaw rate in cd/s
 */
float QuadPlane::get_pilot_input_yaw_rate_cds(void) const
{
    if (plane.get_throttle_input() <= 0 && !plane.auto_throttle_mode) {
        // the user may be trying to disarm
        return 0;
    }

    // add in rudder input
    return plane.channel_rudder->get_control_in() * yaw_rate_max / 45;
}

/*
  get overall desired yaw rate in cd/s
 */
float QuadPlane::get_desired_yaw_rate_cds(void)
{
    float yaw_cds = 0;
    if (reverse_transition_active && is_tailsitter()) {
        return 0.0f;
    }
    if (assisted_flight) {
        // use bank angle to get desired yaw rate
        yaw_cds += desired_auto_yaw_rate_cds();
    }
    if (plane.get_throttle_input() <= 0 && !plane.auto_throttle_mode) {
        // the user may be trying to disarm
        return 0;
    }
    // add in pilot input
    yaw_cds += get_pilot_input_yaw_rate_cds();

    // add in weathervaning
    yaw_cds += get_weathervane_yaw_rate_cds();
    
    return yaw_cds;
}

// get pilot desired climb rate in cm/s
float QuadPlane::get_pilot_desired_climb_rate_cms(void) const
{
    if (plane.failsafe.rc_failsafe || plane.failsafe.throttle_counter > 0) {
        // descend at 0.5m/s for now
        return -50;
    }
    uint16_t dead_zone = plane.channel_throttle->get_dead_zone();
    uint16_t trim = (plane.channel_throttle->get_radio_max() + plane.channel_throttle->get_radio_min())/2;
    float climb_rate_cms = pilot_velocity_z_max * plane.channel_throttle->pwm_to_angle_dz_trim(dead_zone, trim) / 100.0f;
    // enable limiting of sink rate close to ground
    climb_rate_cms = MAX(climb_rate_cms, -_pilot_sink_rate_limit_cms);
    return climb_rate_cms;
}


/*
  initialise throttle_wait based on throttle and is_flying()
 */
void QuadPlane::init_throttle_wait(void)
{
    if (plane.get_throttle_input() >= 10 ||
        plane.is_flying()) {
        throttle_wait = false;
    } else {
        throttle_wait = true;        
    }
}
    
// set motor arming
void QuadPlane::set_armed(bool armed)
{
    if (!initialised) {
        return;
    }
    motors->armed(armed);
}


/*
  estimate desired climb rate for assistance (in cm/s)
 */
float QuadPlane::assist_climb_rate_cms(void) const
{
    float climb_rate;
    if (plane.auto_throttle_mode) {
        // use altitude_error_cm, spread over 10s interval
        climb_rate = plane.altitude_error_cm * 0.1f;
    } else {
        // otherwise estimate from pilot input
        climb_rate = plane.get_throttle_input();
        if (climb_rate >= 0.0f) {
            climb_rate *= plane.g.flybywire_climb_rate * (plane.nav_pitch_cd/(float)plane.aparm.pitch_limit_max_cd);
        } else {
            climb_rate *= plane.g.flybywire_sink_rate * (plane.nav_pitch_cd/(float)plane.aparm.pitch_limit_max_cd);
        }
    }
    climb_rate = constrain_float(climb_rate, -wp_nav->get_speed_down(), wp_nav->get_speed_up());

    // bring in the demanded climb rate over 2 seconds
    const uint32_t ramp_up_time_ms = 2000;
    const uint32_t dt_since_start = last_pidz_active_ms - last_pidz_init_ms;
    if (dt_since_start < ramp_up_time_ms) {
        climb_rate = linear_interpolate(0, climb_rate, dt_since_start, 0, ramp_up_time_ms);
    }
    
    return climb_rate;
}

/*
  calculate desired yaw rate for assistance
 */
float QuadPlane::desired_auto_yaw_rate_cds(void) const
{
    float aspeed;
    if (!ahrs.airspeed_estimate(&aspeed) || aspeed < plane.aparm.airspeed_min) {
        aspeed = plane.aparm.airspeed_min;
    }
    if (aspeed < 1) {
        aspeed = 1;
    }
    float yaw_rate = degrees(GRAVITY_MSS * tanf(radians(plane.nav_roll_cd*0.01f))/aspeed) * 100;
    return yaw_rate;
}

/*
  return true if the quadplane should provide stability assistance
 */
bool QuadPlane::assistance_needed(float aspeed)
{
    if (assist_speed <= 0) {
        // assistance disabled
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return false;
    }
    if (aspeed < assist_speed) {
        // assistance due to Q_ASSIST_SPEED
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return true;
    }

    if (assist_angle <= 0) {
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return false;
    }

    /*
      now check if we should provide assistance due to attitude error
     */

    const uint16_t allowed_envelope_error_cd = 500U;
    if (labs(ahrs.roll_sensor) <= plane.aparm.roll_limit_cd+allowed_envelope_error_cd &&
        ahrs.pitch_sensor < plane.aparm.pitch_limit_max_cd+allowed_envelope_error_cd &&
        ahrs.pitch_sensor > -(allowed_envelope_error_cd-plane.aparm.pitch_limit_min_cd)) {
        // we are inside allowed attitude envelope
        in_angle_assist = false;
        angle_error_start_ms = 0;
        return false;
    }
    
    int32_t max_angle_cd = 100U*assist_angle;
    if ((labs(ahrs.roll_sensor - plane.nav_roll_cd) < max_angle_cd &&
         labs(ahrs.pitch_sensor - plane.nav_pitch_cd) < max_angle_cd)) {
        // not beyond angle error
        angle_error_start_ms = 0;
        in_angle_assist = false;
        return false;
    }
    const uint32_t now = AP_HAL::millis();
    if (angle_error_start_ms == 0) {
        angle_error_start_ms = now;
    }
    bool ret = (now - angle_error_start_ms) >= 1000U;
    if (ret && !in_angle_assist) {
        in_angle_assist = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Angle assist r=%d p=%d",
                                         (int)(ahrs.roll_sensor/100),
                                         (int)(ahrs.pitch_sensor/100));
    }
    return ret;
}

/*
  update for transition from quadplane to fixed wing mode
 */
void QuadPlane::update_transition_to_fw(void)
{
    if (plane.control_mode == MANUAL ||
        plane.control_mode == ACRO ||
        plane.control_mode == TRAINING) {
        // in manual modes quad motors are always off
        if (!tilt.motors_active && !is_tailsitter()) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
            motors->output();
        }
        transition_state = TRANSITION_DONE;
        assisted_flight = false;
        return;
    }

    float aspeed;
    bool have_airspeed = ahrs.airspeed_estimate(&aspeed);

    // tailsitters use angle wait, not airspeed wait
    if (is_tailsitter() && transition_state == TRANSITION_AIRSPEED_WAIT) {
        transition_state = TRANSITION_ANGLE_WAIT_FW;
    }
    
    /*
      see if we should provide some assistance
     */
    if (have_airspeed &&
        assistance_needed(aspeed) &&
        !is_tailsitter() &&
        hal.util->get_soft_armed() &&
        ((plane.auto_throttle_mode && !plane.throttle_suppressed) ||
         plane.get_throttle_input()>0 ||
         plane.is_flying())) {
        // the quad should provide some assistance to the plane
        if (transition_state != TRANSITION_AIRSPEED_WAIT) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition started airspeed %.1f", (double)aspeed);
        }
        transition_state = TRANSITION_AIRSPEED_WAIT;
        transition_start_ms = millis();
        assisted_flight = true;
    } else {
        assisted_flight = false;
    }

    if (is_tailsitter()) {
        if (transition_state == TRANSITION_ANGLE_WAIT_FW &&
            tailsitter_transition_fw_complete()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition FW done");
            transition_state = TRANSITION_DONE;
            fwd_transition = false;

            // reset TECS states prior to hand over to FW control loops
            plane.TECS_controller.vtol_fw_transition_reset();

            // set temporary floor on height demand to prevent unwanted initial diving due to overshoot.
            plane.fw_trans_time_ms = millis();
            plane.fw_trans_alt_floor_cm = plane.relative_target_altitude_cm();
        }
    }
    
    // if rotors are fully forward then we are not transitioning
    if (tiltrotor_fully_fwd()) {
        transition_state = TRANSITION_DONE;
    }
    
    if (transition_state < TRANSITION_TIMER) {
        // set a single loop pitch limit in TECS
        if (plane.ahrs.groundspeed() < 3) {
            // until we have some ground speed limit to zero pitch
            plane.TECS_controller.set_pitch_max_limit(0);
        } else {
            plane.TECS_controller.set_pitch_max_limit(transition_pitch_max);
        }
    } else if (transition_state < TRANSITION_DONE) {
        plane.TECS_controller.set_pitch_max_limit((transition_pitch_max+1)*2);
    }
    if (transition_state < TRANSITION_DONE) {
        // during transition we ask TECS to use a synthetic
        // airspeed. Otherwise the pitch limits will throw off the
        // throttle calculation which is driven by pitch
        plane.TECS_controller.use_synthetic_airspeed();
    }
    
    switch (transition_state) {
    case TRANSITION_AIRSPEED_WAIT: {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        // we hold in hover until the required airspeed is reached
        if (transition_start_ms == 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Transition airspeed wait");
            transition_start_ms = millis();
        }

        if (have_airspeed && aspeed > plane.aparm.airspeed_min && !assisted_flight) {
            transition_start_ms = millis();
            transition_state = TRANSITION_TIMER;
            gcs().send_text(MAV_SEVERITY_INFO, "Transition airspeed reached %.1f", (double)aspeed);
        }
        assisted_flight = true;

        // do not allow a climb on the quad motors during transition
        // a climb would add load to the airframe, and prolongs the
        // transition
        float climb_rate_cms = assist_climb_rate_cms();
        if (options & OPTION_LEVEL_TRANSITION) {
            climb_rate_cms = MIN(climb_rate_cms, 0.0f);
        }
        hold_hover(climb_rate_cms);
        last_throttle = motors->get_throttle();

        // reset integrators while we are below target airspeed as we
        // may build up too much while still primarily under
        // multicopter control
        plane.pitchController.reset_I();
        plane.rollController.reset_I();

        // give full authority to attitude control
        attitude_control->set_throttle_mix_max();
        break;
    }
        
    case TRANSITION_TIMER: {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        // after airspeed is reached we degrade throttle over the
        // transition time, but continue to stabilize
        if (millis() - transition_start_ms > (unsigned)transition_time_ms) {
            transition_state = TRANSITION_DONE;
            gcs().send_text(MAV_SEVERITY_INFO, "Transition done");
        }
        float trans_time_ms = (float)transition_time_ms.get();
        float transition_scale = (trans_time_ms - (millis() - transition_start_ms)) / trans_time_ms;
        float throttle_scaled = last_throttle * transition_scale;

        // set zero throttle mix, to give full authority to
        // throttle. This ensures that the fixed wing controllers get
        // a chance to learn the right integrators during the transition
        attitude_control->set_throttle_mix_value(0.5*transition_scale);

        if (throttle_scaled < 0.01) {
            // ensure we don't drop all the way to zero or the motors
            // will stop stabilizing
            throttle_scaled = 0.01;
        }
        assisted_flight = true;
        hold_stabilize(throttle_scaled);
        break;
    }

    case TRANSITION_ANGLE_WAIT_FW: {
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        if (!fwd_transition) {
            fwd_transition = true;
            nav_pitch_init_cd = ahrs_view->pitch_sensor;
        }

        assisted_flight = true;
        // calculate transition rate in degrees per
        // millisecond. Assume we want to get to the transition angle
        // in half the transition time
        float transition_rate = tailsitter.transition_angle / float(transition_time_ms/2);
        uint32_t dt = AP_HAL::millis() - transition_start_ms;
        plane.nav_pitch_cd = constrain_float(nav_pitch_init_cd - (transition_rate * dt)*100, -8500, 0);
        plane.nav_roll_cd = 0;
        check_attitude_relax();
        if (control_loss_declared) {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0,0,0.0f);
        } else {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                      ahrs_view->pitch_sensor,
                                                                      0);}
        attitude_control->set_throttle_out((motors->get_throttle_hover() + 0.1f), true, 0);
        break;
    }

    case TRANSITION_ANGLE_WAIT_VTOL:
        break;

    case TRANSITION_DONE:
        if (!tilt.motors_active && !is_tailsitter()) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
            motors->output();
        }
        return;
    }

    motors_output();
}

/*
  update motor output for quadplane
 */
void QuadPlane::update(void)
{
    if (!setup()) {
        return;
    }
    
    if (plane.afs.should_crash_vehicle()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        motors->output();
        return;
    }
    
    if (motor_test.running) {
        motor_test_output();
        return;
    }

    bool soft_arm_status = hal.util->get_soft_armed();
    if (!soft_arm_status) {
        /*
          make sure we don't have any residual control from previous flight stages
         */
        if (!is_tailsitter()) {
            // tailsitter attitude controllers don't relax so users can test disarmed in Q modes
            attitude_control->relax_attitude_controllers();
        }
        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_alt_hold_controllers(0);
    } else if (is_tailsitter() && soft_arm_status && !soft_arm_status_prev) {
        // ensure that tailsitter attitude controllers are reset on the first frame when arming
        attitude_control->relax_attitude_controllers();
    }

    // Initialise takeoff states when arming becasue we may have spent significant time armed prior to takeoff
    if (soft_arm_status && !soft_arm_status_prev) {
        init_takeoff_this_frame = true;
    } else {
        init_takeoff_this_frame = false;
    }
    soft_arm_status_prev = soft_arm_status;

    // Initialise controller states in perparation for a takeoff when the vehicle arms
    if (init_takeoff_this_frame) {
        do_vtol_takeoff(plane.mission.get_current_nav_cmd());
    }

    check_yaw_reset();
    
    if (!in_vtol_mode()) {
        update_transition_to_fw();
        _fw_throttle_factor = motors->calc_fwd_compensation_gain();
    } else {
        const uint32_t now = AP_HAL::millis();

        assisted_flight = false;
        
        // give full authority to attitude control
        attitude_control->set_throttle_mix_max();

        // output to motors
        motors_output();

        // When entering a VTOL mode with motors armed go through the normal back transition procesure unless recvering from loss of control.
        // When disarmed, we want to transition immediately to facilitate rapid mode selection prior to flight.
        if (now - last_vtol_mode_ms > 1000 &&
                is_tailsitter() &&
                transition_state != TRANSITION_ANGLE_WAIT_VTOL &&
                plane.arming.is_armed()) {
            if (!control_loss_declared) {
                /*
                  we are just entering a VTOL mode as a tailsitter, set
                  the transition state and commence the initial pull-up and slowdown
                 */
                transition_state = TRANSITION_ANGLE_WAIT_VTOL;
                transition_start_ms = now;
                reverse_transition_pullup_active = true;
                reverse_transition_active = true;
                gcs().send_text(MAV_SEVERITY_INFO, "Transition VTOL - pullup start");
            } else {
                gcs().send_text(MAV_SEVERITY_INFO, "Transition VTOL - emergency recovery");
                // Emergency loss of control recovery so switch straight to VTOL mode and setup for transition back to fixed wing
                transition_start_ms = now;
                transition_state = TRANSITION_ANGLE_WAIT_FW;
                reverse_transition_active =  false;
                init_mode();
            }
        } else if (is_tailsitter() &&
                   transition_state == TRANSITION_ANGLE_WAIT_VTOL) {
            if (reverse_transition_pullup_active && tailsitter_transition_pullup_complete()) {
                /*
                  we have completed the FW pullup
                */
                gcs().send_text(MAV_SEVERITY_INFO, "Transition VTOL - brake start");
                transition_start_ms = now;
                reverse_transition_pullup_active = false;
            } else if (reverse_transition_active && tailsitter_transition_vtol_complete()) {
                /*
                  we have completed transition to VTOL as a tailsitter,
                  setup for the reverse transition when needed
                */
                gcs().send_text(MAV_SEVERITY_INFO, "Transition VTOL - complete");
                transition_state = TRANSITION_ANGLE_WAIT_FW;
                transition_start_ms = now;
                reverse_transition_active =  false;
                init_mode();
            }
        } else {
            /*
              setup the transition state appropriately for next time we go into a non-VTOL mode
            */
            transition_start_ms = 0;
            if (throttle_wait && !plane.is_flying()) {
                transition_state = TRANSITION_DONE;
            } else if (is_tailsitter()) {
                /*
                  setup for the transition back to fixed wing for later
                 */
                transition_state = TRANSITION_ANGLE_WAIT_FW;
                transition_start_ms = now;
            } else {
                /*
                  setup for airspeed wait for later
                 */
                transition_state = TRANSITION_AIRSPEED_WAIT;
            }
            last_throttle = motors->get_throttle();
        }
            
        last_vtol_mode_ms = now;
    }

    // disable throttle_wait when throttle rises above 10%
    if (throttle_wait &&
        (plane.get_throttle_input() > 10 ||
         plane.failsafe.rc_failsafe ||
         plane.failsafe.throttle_counter>0)) {
        throttle_wait = false;
    }

    tiltrotor_update();
}

/*
  see if motors should be shutdown. If they should be then change AP_Motors state to 
  AP_Motors::DESIRED_SHUT_DOWN

  This is a safety check to prevent accidental motor runs on the
  ground, such as if RC fails and QRTL is started
 */
void QuadPlane::check_throttle_suppression(void)
{
    // if the motors have been running in the last 2 seconds then
    // allow them to run now
    if (AP_HAL::millis() - last_motors_active_ms < 2000) {
        return;
    }

    // see if motors are already disabled
    if (motors->get_desired_spool_state() < AP_Motors::DESIRED_THROTTLE_UNLIMITED) {
        return;
    }

    // if the users throttle is above zero then allow motors to run
    if (plane.get_throttle_input() != 0) {
        return;
    }

    // if we are in a fixed wing auto throttle mode and we have
    // unsuppressed the throttle then allow motors to run
    if (plane.auto_throttle_mode && !plane.throttle_suppressed) {
        return;
    }

    // if our vertical velocity is greater than 1m/s then allow motors to run
    if (fabsf(inertial_nav.get_velocity_z()) > 100) {
        return;
    }

    // if we are more than 5m from home altitude then allow motors to run
    if (plane.relative_ground_altitude(plane.g.rangefinder_landing) > 5) {
        return;
    }

    // allow for takeoff
    if (plane.control_mode == AUTO && is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)) {
        return;
    }
    
    // motors should be in the spin when armed state to warn user they could become active
    motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
    motors->set_throttle(0);
    last_motors_active_ms = 0;
}

/*
  output motors and do any copter needed
 */
void QuadPlane::motors_output(bool run_rate_controller)
{
    if (run_rate_controller) {
        attitude_control->rate_controller_run();
    }

    if (!hal.util->get_soft_armed() || plane.afs.should_crash_vehicle()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        motors->output();
        return;
    }
    if (esc_calibration && AP_Notify::flags.esc_calibration && plane.control_mode == QSTABILIZE) {
        // output is direct from run_esc_calibration()
        return;
    }

    // see if motors should be shut down
    check_throttle_suppression();
    
    motors->output();
    if (motors->armed()) {
        plane.DataFlash.Log_Write_Rate(plane.ahrs, *motors, *attitude_control, *pos_control);
        Log_Write_QControl_Tuning();
        const uint32_t now = AP_HAL::millis();
        if (now - last_ctrl_log_ms > 100) {
            attitude_control->control_monitor_log();
        }
    }

    // remember when motors were last active for throttle suppression
    if (motors->get_throttle() > 0.01f || tilt.motors_active) {
        last_motors_active_ms = AP_HAL::millis();
    }
    
}

/*
  update control mode for quadplane modes
 */
void QuadPlane::control_run(void)
{
    if (!initialised) {
        return;
    }

    // height above ground is required in many places
    _height_above_ground_m = plane.relative_ground_altitude(plane.g.rangefinder_landing);

    // run logic used to determine limits based on positon and height relative to home location
    launch_recovery_zone_logic();

    switch (plane.control_mode) {
    case QSTABILIZE:
        control_stabilize();
        break;
    case QHOVER:
        control_hover();
        break;
    case QLOITER:
        control_loiter();
        break;
    case QLAND:
        control_qland();
        break;
    case QRTL:
        control_qrtl();
        break;
    default:
        break;
    }
 }

/*
 run corvo launch and recovery zone logic
 */
void QuadPlane::launch_recovery_zone_logic(void) {
    // Addtional protections required to enable one hand operation
    if ((tailsitter.input_type == plane.quadplane.TAILSITTER_CORVOX) && RC_Channels::has_active_overrides()) {
        if (!motors->armed()) {
            // reset flags when disarmed
            _doing_takeoff_jump = false;
            _reached_rtl_alt = false;
            _outside_takeoff_zone = false;
            _land_point_offset_NE.zero();
        } else {
            // corvo X uses up button press to arm and start a takeoff, followed by automatic climb to height set by Q_TVBS_JMP_ALT
            if ((plane.control_mode == QLOITER) && motors->armed() && !_prev_arm_status) {
                // start the jump to Q_RTL_ALT height
                set_alt_target_current();
                _doing_takeoff_jump = true;
            } else if (_doing_takeoff_jump &&
                       ((plane.control_mode != QLOITER)
                        || (plane.relative_altitude > (float)tailsitter.tvbs_jmp_alt)
                        || (get_pilot_desired_climb_rate_cms() < -50))) {
                // jump stops when height is reached, the mode changes or the pilot commands a descent
                set_alt_target_current();
                _doing_takeoff_jump = false;
            }

            // check takeoff height clearance - used to prevent early transition into FW flight modes
            if (!_reached_rtl_alt && (_height_above_ground_m > (float)qrtl_alt)) {
                _reached_rtl_alt = true;
            }

            // check outside zone using hysteresis set to typical GPS drift value to avoid rapid switching
            Vector2f posNE_rel_to_home;
            if (ahrs.get_relative_position_NE_home(posNE_rel_to_home)){
                float distance_to_home = posNE_rel_to_home.length();
                if (!_outside_takeoff_zone && (distance_to_home > ((float)tailsitter.tvbs_jmp_radius + 5.0f))) {
                    _outside_takeoff_zone =  true;
                } else if (_outside_takeoff_zone && (distance_to_home < (float)tailsitter.tvbs_jmp_radius)) {
                    _outside_takeoff_zone =  false;
                }
            }

            // generate descent rate to be used when descend button is pressed
            if (_outside_takeoff_zone) {
                _pilot_sink_rate_limit_cms = pilot_velocity_z_max;
            } else if (_height_above_ground_m > land_final_alt) {
                _pilot_sink_rate_limit_cms = linear_interpolate(land_speed_cms, pilot_velocity_z_max,
                                                               plane.relative_altitude,
                                                               land_final_alt, land_final_alt+6);
            } else {
                _pilot_sink_rate_limit_cms = linear_interpolate((0.7f * land_speed_cms), land_speed_cms,
                                                               plane.relative_altitude,
                                                               MIN(2.0f,(land_final_alt-1.0f)), land_final_alt);
            }

            // toggle auto land arrest using climb/descend buttons
            if (plane.channel_throttle->norm_input() < 0.5f) {
                _climb_start_event_ms = AP_HAL::millis();
            }
            if (plane.channel_throttle->norm_input() > -0.5f) {
                _descend_start_event_ms = AP_HAL::millis();
            }
            if (!_auto_land_arrested && ((AP_HAL::millis() - _climb_start_event_ms) > 1000)) {
                _auto_land_arrested = true;
            } else if (_auto_land_arrested && (((AP_HAL::millis() - _descend_start_event_ms) > 1000) || !motors->armed())) {
                _auto_land_arrested = false;
            }

        }
    } else {
        // defaults when operating with normal RC handset removes restrictions and disables takeoff jump
        _doing_takeoff_jump = false;
        _reached_rtl_alt = true;
        _outside_takeoff_zone = true;
        _pilot_sink_rate_limit_cms = pilot_velocity_z_max;
    }

    // record value for next frame
    _prev_arm_status = motors->armed();
}

// return true if transition to a forward flight mode is allowed
bool QuadPlane::fw_transition_allowed(void) const
{
    // Until we have reached the altitude set by Q_RTL_ALT for the first time, no forward trnsitoin is allowed
    bool ret = false;
    if (_reached_rtl_alt) {
        // Once we have passed the height we must be outside the recovery zone set by Q_RTL_ALT and Q_TVBS_JMP_RAD. This allows for hilltop operations.
        ret = (_outside_takeoff_zone || (_height_above_ground_m > (float)qrtl_alt));
    }
    return ret;
}

/*
  enter a quadplane mode
 */
bool QuadPlane::init_mode(void)
{
    if (!setup()) {
        return false;
    }
    if (!initialised) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "QuadPlane mode refused");
        return false;
    }

    AP_Notify::flags.esc_calibration = false;

    switch (plane.control_mode) {
    case QSTABILIZE:
        init_stabilize();
        break;
    case QHOVER:
        init_hover();
        break;
    case QLOITER:
        init_loiter();
        break;
    case QLAND:
        init_qland();
        break;
    case QRTL:
        init_qrtl();
        break;
    case GUIDED:
        guided_takeoff = false;
        break;
    default:
        break;
    }
    return true;
}

/*
  handle a MAVLink DO_VTOL_TRANSITION
 */
bool QuadPlane::handle_do_vtol_transition(enum MAV_VTOL_STATE state)
{
    if (!available()) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "VTOL not available");
        return false;
    }
    if (plane.control_mode != AUTO) {
        gcs().send_text(MAV_SEVERITY_NOTICE, "VTOL transition only in AUTO");
        return false;
    }
    switch (state) {
    case MAV_VTOL_STATE_MC:
        if (!plane.auto_state.vtol_mode) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Entered VTOL mode");
        }
        plane.auto_state.vtol_mode = true;
        return true;
        
    case MAV_VTOL_STATE_FW:
        if (plane.auto_state.vtol_mode) {
            gcs().send_text(MAV_SEVERITY_NOTICE, "Exited VTOL mode");
        }
        plane.auto_state.vtol_mode = false;

        return true;

    default:
        break;
    }

    gcs().send_text(MAV_SEVERITY_NOTICE, "Invalid VTOL mode");
    return false;
}

/*
  are we in a VTOL auto state?
 */
bool QuadPlane::in_vtol_auto(void) const
{
    if (!enable) {
        return false;
    }
    if (plane.control_mode != AUTO) {
        return false;
    }
    if (plane.auto_state.vtol_mode) {
        return true;
    }
    uint16_t id = plane.mission.get_current_nav_cmd().id;
    switch (id) {
    case MAV_CMD_NAV_VTOL_TAKEOFF:
        return true;
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_LOITER_TO_ALT:
        return plane.auto_state.vtol_loiter;
    case MAV_CMD_NAV_TAKEOFF:
        return is_vtol_takeoff(id);
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
        return is_vtol_land(id);
    default:
        return false;
    }
}

/*
  are we in a VTOL mode?
 */
bool QuadPlane::in_vtol_mode(void) const
{
    if (!enable) {
        return false;
    }
    return (plane.control_mode == QSTABILIZE ||
            plane.control_mode == QHOVER ||
            plane.control_mode == QLOITER ||
            plane.control_mode == QLAND ||
            plane.control_mode == QRTL ||
            ((plane.control_mode == GUIDED || plane.control_mode == AVOID_ADSB) && plane.auto_state.vtol_loiter) ||
            in_vtol_auto());
}

// not in a mode suitable for corvo X to takeoff
bool QuadPlane::corvo_takeoff_inhibit(void) const
{
    if (tailsitter.input_type != TAILSITTER_CORVOX) {
        return false;
    }

    // VTOL functions not enabled
    if (!enable) {
        return true;
    }

    // not in a suitable flight mode
    if (plane.control_mode != QLOITER && !in_vtol_auto()) {
        return true;
    }

    return false;
}

/*
  main landing controller. Used for landing and RTL.
 */
void QuadPlane::vtol_position_controller(void)
{
    if (!setup()) {
        return;
    }

    setup_target_position();

    const Location &loc = plane.next_WP_loc;

    check_attitude_relax();

    // Calculate delta time and detect time slip or first frame
    float dt = 0.001f * (float)(AP_HAL::millis() - poscontrol.time_ms);
    poscontrol.time_ms = AP_HAL::millis();
    if (!poscontrol.initialised || (dt > 0.1f)) {
        dt = 0.0f;
    }

    // Prevent strong winds dragging the vehicle downwind due to the pos control internally calculated
    // leash length being too small.
    pos_control->set_max_accel_xy(1000.0f);
    pos_control->set_max_speed_xy(100.0f * pos_control->get_max_fwd_airspd());

    // we are going to need target altitude in several calculations
    float target_altitude_cm = plane.next_WP_loc.alt;

    // check if definitely below the target altitude - allow for height fluctuation
    bool too_low = (0.01f*(target_altitude_cm - plane.home.alt) > _height_above_ground_m + 0.5f);

    // determine if inside or outside the landing recovery cone using hysteresis
    float cone_radius = (float)tailsitter.tvbs_land_cone_radius + (_height_above_ground_m / atanf(radians(constrain_float((float)tailsitter.tvbs_land_cone_elev, 0.0f, 80.0f))));
    if (poscontrol.radial_error < (cone_radius - 0.5f)) {
        poscontrol.outside_cone = false;
    } else if (poscontrol.radial_error > (cone_radius + 0.5f)) {
        poscontrol.outside_cone = true;
    }

    // horizontal position control
    switch (poscontrol.state) {

    case QPOS_POSITION1: {
        // run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, loc);

        Vector2f diff_wp = location_diff(plane.current_loc, loc);
        float distance = diff_wp.length();

        // calculate position unit vector and speed towards target
        Vector2f groundspeed = ahrs.groundspeed_vector();
        Vector2f unit_pos_to_target = diff_wp.normalized();
        float speed_towards_target = distance>1?(unit_pos_to_target * groundspeed):0;

        if (!poscontrol.initialised) {
            poscontrol.initialised = true;

            // initialise scaling so we start off targeting our
            // current linear speed towards the target. If this is
            // less than the wpnav speed then the wpnav speed is used
            // land_speed_scale is then used to linearly change
            // velocity as we approach the waypoint, aiming for zero
            // speed at the waypoint
            // setup land_speed_scale so at current distance we
            // maintain speed towards target, and slow down as we
            // approach

            // max_speed will control how fast we will fly. It will always decrease
            poscontrol.max_speed = MAX(speed_towards_target, wp_nav->get_speed_xy() * 0.01f);

            // initialise the target velocity to the current vehicle velocity
            poscontrol.target_velocity = ahrs.groundspeed_vector();

            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position1 started v=%.1f d=%.1f",
                                    (double)speed_towards_target, (double)plane.auto_state.wp_distance);
         }

        // get previously demanded inbound velocity
        float velocity_inbound_demanded =  unit_pos_to_target.x*poscontrol.target_velocity.x + unit_pos_to_target.y*poscontrol.target_velocity.y;

        // calculate distance required to stop
        const float accel_limit = 2.0f;
        const float decel_limit = 1.0f;
        float stopping_distance_m = sq(velocity_inbound_demanded) / ( 2.0f * decel_limit);

        // accelerate until max speed is reached or stopping margin test fails
        bool stopping_distance_pass = (stopping_distance_m < (distance + 1.0f)) || (speed_towards_target < 0.0f);

        Vector2f accel_XY_cmss;
        if (stopping_distance_pass) {
            // accelerate velocity vector from current value to value  inbound to target point
            // observing acceleration and velocity limits
            Vector2f vel_target = unit_pos_to_target * (wp_nav->get_speed_xy()*0.01f);
            // if haven't reached RTL height, wait
            if (too_low && poscontrol.outside_cone) {
                vel_target.zero();
            }
            Vector2f vel_delta = vel_target - poscontrol.target_velocity;
            float max_delta = accel_limit * dt;
            if (vel_delta.length() > max_delta) {
                // accelerate at a constant rate until velocity demand target achieved
                // calculate velocity demand
                vel_delta = vel_delta.normalized() * max_delta;
                poscontrol.target_velocity += vel_delta;

                // calculate acceleration demand in forward,right axis frame
                Vector2f accel_NE = unit_pos_to_target * accel_limit;
                accel_XY_cmss.x = (accel_NE.x * ahrs_view->cos_yaw() + accel_NE.y * ahrs_view->sin_yaw()) * 100.0f;
                accel_XY_cmss.y = (accel_NE.y * ahrs_view->cos_yaw() - accel_NE.x * ahrs_view->sin_yaw()) * 100.0f;
            } else {
                // maintain constant velocity target
                poscontrol.target_velocity = vel_target;
                accel_XY_cmss.zero();
            }
        } else {
            // perform a constant decel brake manoeuvre
            float inbound_speed_target = constrain_float(sqrtf(2.0f * decel_limit * distance),0.0f,wp_nav->get_speed_xy()*0.01f);
            poscontrol.target_velocity = unit_pos_to_target * inbound_speed_target;

            // calculate acceleration demand in forward,right axis frame
            Vector2f accel_NE = unit_pos_to_target * (-decel_limit);
            accel_XY_cmss.x = (accel_NE.x * ahrs_view->cos_yaw() + accel_NE.y * ahrs_view->sin_yaw()) * 100.0f;
            accel_XY_cmss.y = (accel_NE.y * ahrs_view->cos_yaw() - accel_NE.x * ahrs_view->sin_yaw()) * 100.0f;
        }

        // Output the feed forward acceleration
        pos_control->set_desired_accel_xy(accel_XY_cmss.x,accel_XY_cmss.y);

        // demand a target velocity
        pos_control->set_desired_velocity_xy(poscontrol.target_velocity.x*100,
                                             poscontrol.target_velocity.y*100);

        // reset position controller xy target to current position
        // because we only want velocity control (no position control)
        const Vector3f& curr_pos = inertial_nav.get_position();
        pos_control->set_xy_target(curr_pos.x, curr_pos.y);

        // run horizontal velocity controller
        pos_control->update_vel_controller_xy();

        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll();
        if (frame_class == AP_Motors::MOTOR_FRAME_TVBS) {
            plane.nav_roll_cd = (int32_t)(tailsitter.tvbs_roll_gain * (float)plane.nav_roll_cd);
        }
        plane.nav_pitch_cd = pos_control->get_pitch();

        if (!is_tailsitter()) {
            /*
              limit the pitch down with an expanding envelope. This
              prevents the velocity controller demanding nose down during
              the initial slowdown if the target velocity curve is higher
              than the actual velocity curve (for a high drag
              aircraft). Nose down will cause a lot of downforce on the
              wings which will draw a lot of current and also cause the
              aircraft to lose altitude rapidly.
             */
            float pitch_limit_cd = linear_interpolate(-300, plane.aparm.pitch_limit_min_cd,
                                                      plane.auto_state.wp_proportion, 0, 1);
            if (plane.nav_pitch_cd < pitch_limit_cd) {
                plane.nav_pitch_cd = pitch_limit_cd;
                // tell the pos controller we have limited the pitch to
                // stop integrator buildup
                pos_control->set_limit_accel_xy();
            }
        }
        
        // provide set points to the attitude controller
        if (control_loss_declared || reverse_transition_active) {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0,0,0.0f);
        } else {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                             ahrs_view->pitch_sensor,
                                                                             desired_auto_yaw_rate_cds() + get_weathervane_yaw_rate_cds());
        }
        if (plane.auto_state.wp_proportion >= 1 ||
            ((distance < 5) && ((speed_towards_target < 0.0f) || (1.0f * distance < sqrtf(2.0f * decel_limit * distance))))) {
            poscontrol.state = QPOS_POSITION2;
            loiter_nav->clear_pilot_desired_acceleration();
            loiter_nav->init_target();
            pos_control->set_desired_accel_xy(0.0f,0.0f);
            pos_control->set_desired_velocity_xy(0.0f,0.0f);
            gcs().send_text(MAV_SEVERITY_INFO,"VTOL position2 started v=%.1f d=%.1f",
                                    (double)speed_towards_target, (double)plane.auto_state.wp_distance);
        }
        break;
    }

    case QPOS_POSITION2:
    case QPOS_LAND_DESCEND:
    {
        // for final land repositioning and descent we run the position controller
        // also run fixed wing navigation
        plane.nav_controller->update_waypoint(plane.prev_WP_loc, loc);
    }
    FALLTHROUGH;

    case QPOS_LAND_FINAL:
    {
        // set position controller desired velocity and acceleration to zero
        if (!_auto_land_arrested) {
           // handle special case where operator is adjusting the landing waypoint
            pos_control->set_desired_velocity_xy(0.0f,0.0f);
        }
        pos_control->set_desired_accel_xy(0.0f,0.0f);

        // set position control target and update
        // also let controller know to expect touchdown
        pos_control->set_xy_target(poscontrol.target.x, poscontrol.target.y);
        if (poscontrol.state == QPOS_LAND_FINAL) {
            pos_control->set_vtol_landing_expected();
        }
        pos_control->update_xy_controller();

        // nav roll and pitch are controller by position controller
        plane.nav_roll_cd = pos_control->get_roll();
        plane.nav_pitch_cd = pos_control->get_pitch();

        // provide set points to the attitude controller
        if (control_loss_declared) {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0,0,0.0f);
        } else {
            attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                          ahrs_view->pitch_sensor,
                                                                          get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds());
        }
        break;
    }

    case QPOS_LAND_COMPLETE:
        // nothing to do
        break;
    }

    // now height control

    switch (poscontrol.state) {
    case QPOS_POSITION1:
    case QPOS_POSITION2: {
        bool vtol_loiter_auto = false;
        if (plane.control_mode == AUTO) {
            switch (plane.mission.get_current_nav_cmd().id) {
            case MAV_CMD_NAV_LOITER_UNLIM:
            case MAV_CMD_NAV_LOITER_TIME:
            case MAV_CMD_NAV_LOITER_TURNS:
            case MAV_CMD_NAV_LOITER_TO_ALT:
                vtol_loiter_auto = true;
                break;
            }
        }
        if (plane.control_mode == GUIDED || vtol_loiter_auto) {
            plane.ahrs.get_position(plane.current_loc);
            if (poscontrol.slow_descent) {
                // gradually descend as we approach target
                plane.auto_state.wp_proportion = location_path_proportion(plane.current_loc, 
                                                                          plane.prev_WP_loc, plane.next_WP_loc);
                target_altitude_cm = linear_interpolate(plane.prev_WP_loc.alt,
                                                     plane.next_WP_loc.alt,
                                                     plane.auto_state.wp_proportion,
                                                     0, 1);
            }
            adjust_alt_target(target_altitude_cm - plane.home.alt);
        } else if (plane.control_mode == QRTL || plane.control_mode == QLAND) {
            plane.ahrs.get_position(plane.current_loc);
            if (poscontrol.slow_descent) {
                // gradually descend as we approach target
                plane.auto_state.wp_proportion = location_path_proportion(plane.current_loc,
                                                                          plane.prev_WP_loc, plane.next_WP_loc);
                target_altitude_cm = linear_interpolate(plane.prev_WP_loc.alt,
                                                     plane.next_WP_loc.alt,
                                                     plane.auto_state.wp_proportion,
                                                     0, 1);
                adjust_alt_target(target_altitude_cm - plane.home.alt);
            } else {
                // allow descent if inside the landing cone
                if (poscontrol.outside_cone) {
                    if (plane.control_mode == QRTL) {
                        // handle special case of RTL where we want to climb to the RTL height if outside the landing cone
                        if (0.01f * (target_altitude_cm - plane.home.alt) > _height_above_ground_m) {
                            // climb up to RTL altitude
                            pos_control->set_alt_target_from_climb_rate(pilot_velocity_z_max, plane.G_Dt, false);
                        } else {
                            // stay above RTL altitude until inside cone
                            pos_control->set_alt_target_from_climb_rate(0, plane.G_Dt, false);
                        }
                    } else {
                        pos_control->set_alt_target_from_climb_rate(0, plane.G_Dt, false);
                    }
                } else {
                    pos_control->set_alt_target_from_climb_rate(-landing_descent_rate_cms(_height_above_ground_m), plane.G_Dt, true);
                }
            }
        }
        break;
    }

    case QPOS_LAND_DESCEND: {
        if (_auto_land_arrested) {
            // allow pilot to raise alt during repositioning
            pos_control->set_alt_target_from_climb_rate(MAX(0, get_pilot_desired_climb_rate_cms()), plane.G_Dt, false);
        } else if (poscontrol.outside_cone && (descent_delay_time_sec < 30.0f)) {
            // don't allow descent if outside the landing cone
            pos_control->set_alt_target_from_climb_rate(0, plane.G_Dt, false);
            descent_delay_time_sec += dt;
        } else {
            pos_control->set_alt_target_from_climb_rate(-landing_descent_rate_cms(_height_above_ground_m), plane.G_Dt, true);
        }
        break;
    }

    case QPOS_LAND_FINAL: {
        if (_auto_land_arrested) {
            // allow pilot to raise alt during repositioning
            pos_control->set_alt_target_from_climb_rate(MAX(0, get_pilot_desired_climb_rate_cms()), plane.G_Dt, false);
        } else if ((!weathervane.tip_warning && !poscontrol.outside_cone) || descent_delay_time_sec > 30.0f) {
            // complete descent if landing conditions are met or we have delayed the descent for more than 30 seconds in total
            // slow further to half the kinetic energy at the expected touchdown point allowing for expected 2m height error
            float kinetic_energy_ratio = linear_interpolate(0.5f, 1.0f,
                                              _height_above_ground_m,
                                              2.0f, MAX(land_final_alt, 2.0f));
            float hgt_rate_dem_cms = - sqrtf(kinetic_energy_ratio) * (float)land_speed_cms;
            pos_control->set_alt_target_from_climb_rate(hgt_rate_dem_cms, plane.G_Dt, true);
        } else {
            // conditions not good for touchdown so arrest descent
            pos_control->set_alt_target_from_climb_rate(0, plane.G_Dt, true);
            descent_delay_time_sec += dt;
        }
        break;
    }
        
    case QPOS_LAND_COMPLETE:
        break;
    }
    
    run_z_controller();
}


/*
  setup the target position based on plane.next_WP_loc
  adjust using pilot stick inputs if auto landing has been arrested
 */
void QuadPlane::setup_target_position(void)
{
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    const uint32_t now = AP_HAL::millis();
    uint32_t dt_ms = now - last_loiter_ms;
    last_loiter_ms = now;

    // Get waypoint and compare with internally stored value. If different, then the waypoint has been
    // modified externally and th epositon controller destination needs to be reset.
    if (!locations_are_same(plane.next_WP_loc, last_auto_target) ||
        plane.next_WP_loc.alt != last_auto_target.alt ||
        dt_ms > 500) {
        wp_nav->set_wp_destination(poscontrol.target);
    }
    last_auto_target = plane.next_WP_loc;

    // During descent and final landing, the pilot can adjust the landibng position using control sticks
    if (_auto_land_arrested && (dt_ms < 200) && ((poscontrol.state == QPOS_LAND_DESCEND) || (poscontrol.state == QPOS_LAND_FINAL))) {
        // allow pilot to adjust waypoint at a slow speed - this is only intended for fine adjustment
        const float stick_to_speed = (2.0f / 4500.0f);
        float velY = stick_to_speed * plane.channel_roll->get_control_in();
        float velX = - stick_to_speed * plane.channel_pitch->get_control_in();
        float vel_N = velX * ahrs_view->cos_yaw() - velY * ahrs_view->sin_yaw();
        float vel_E = velX * ahrs_view->sin_yaw() + velY * ahrs_view->cos_yaw();
        pos_control->set_desired_velocity_xy(100.0f * vel_N, 100.0f * vel_E);
        float dt_sec = 0.001f * (float)dt_ms;
        _land_point_offset_NE.x += dt_sec * vel_N;
        _land_point_offset_NE.y += dt_sec * vel_E;
    } else if ((poscontrol.state != QPOS_LAND_DESCEND) && (poscontrol.state != QPOS_LAND_FINAL)) {
        _auto_land_arrested = false;
        _land_point_offset_NE.zero();
    }

    // Update the position control target including pilot adjustments
    Location origin = inertial_nav.get_origin();
    Vector2f WP_pos_adj_NE = location_diff(origin, plane.next_WP_loc);
    WP_pos_adj_NE += _land_point_offset_NE;
    poscontrol.target.x = WP_pos_adj_NE.x * 100;
    poscontrol.target.y = WP_pos_adj_NE.y * 100;
    poscontrol.target.z = plane.next_WP_loc.alt - origin.alt;

    // calculate radial distance to waypoint
    Vector2f pos_error_NE = location_diff(origin, plane.current_loc) - WP_pos_adj_NE;
    poscontrol.radial_error = pos_error_NE.length();
    
    // setup vertical speed and acceleration
    pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_max_accel_z(pilot_accel_z);
}


/*
  run takeoff controller to climb vertically
 */
void QuadPlane::takeoff_controller(void)
{
    // for takeoff we use the position controller
    check_attitude_relax();

    // ensure motor throttle limits are removed
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // set position controller desired velocity and acceleration to zero and run position controller
    // position is set in do_vtol_takeoff()
    pos_control->set_desired_velocity_xy(0.0f,0.0f);
    pos_control->set_desired_accel_xy(0.0f,0.0f);

    // Prevent strong winds dragging the vehicle downwind from the takeoff point due to the pos control internally calculated
    // leash length being too small. Setting the pos control accel limit and speed limit higher prevents this and is OK becasue
    // we are starting at the correct position.
    pos_control->set_max_accel_xy(1000.0f);
    pos_control->set_max_speed_xy(100.0f * pos_control->get_max_fwd_airspd());

    // set position control target and update
    pos_control->set_xy_target(takeoff_pos_cm.x, takeoff_pos_cm.y);
    pos_control->update_xy_controller();

    // nav roll and pitch are taken from the position controller
    plane.nav_roll_cd = pos_control->get_roll();
    if (frame_class == AP_Motors::MOTOR_FRAME_TVBS) {
        plane.nav_roll_cd = (int32_t)(tailsitter.tvbs_roll_gain * (float)plane.nav_roll_cd);
    }
    plane.nav_pitch_cd = pos_control->get_pitch();

    // provide set points to the attitude controller
    if (control_loss_declared) {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0,0,0.0f);
    } else {
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(plane.nav_roll_cd,
                                                                      ahrs_view->pitch_sensor,
                                                                      get_pilot_input_yaw_rate_cds() + get_weathervane_yaw_rate_cds());
    }

    pos_control->add_takeoff_climb_rate(pilot_velocity_z_max, plane.G_Dt);
    run_z_controller();

    // trigger a position controller wind drift state reset if vehicle drits outside a +- 45 degree cone centred on the takeoff location.
    // also scale up wind drift integrator gain during first part of ascent to speed learning
    const Vector3f& curr_pos = inertial_nav.get_position();
    float height_cm = plane.current_loc.alt - takeoff_alt_cm;
    float gain_scaler = tailsitter.tvbs_to_scaler * (1.0f - constrain_float(height_cm / 1000.0f, 0.0f, 1.0f));
    pos_control->scale_wind_drift_integ_gain(gain_scaler);
    float pos_err_N_cm = takeoff_pos_cm.x - curr_pos.x;
    float pos_err_E_cm = takeoff_pos_cm.y - curr_pos.y;
    float pos_err_mag_cm = sqrtf(pos_err_N_cm * pos_err_N_cm + pos_err_E_cm * pos_err_E_cm);
    float pos_err_lim_cm = MAX(height_cm * tanf(radians(45.0f)), 200.0f);
    if (!takeoff_reset_complete && (pos_err_mag_cm > pos_err_lim_cm)) {
        pos_control->reset_wind_drift_integ();
        takeoff_reset_complete = true;
        gcs().send_text(MAV_SEVERITY_INFO,"Takeoff drift reset");
    }
}

/*
  run waypoint controller between prev_WP_loc and next_WP_loc
 */
void QuadPlane::waypoint_controller(void)
{
    _auto_land_arrested = false;
    _land_point_offset_NE.zero();
    setup_target_position();

    check_attitude_relax();

    /*
      this is full copter control of auto flight
    */
    // run wpnav controller
    wp_nav->update_wpnav();
    
    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(),
                                                       wp_nav->get_pitch(),
                                                       wp_nav->get_yaw(),
                                                       true);
    // nav roll and pitch are controller by loiter controller
    plane.nav_roll_cd = wp_nav->get_roll();
    if (frame_class == AP_Motors::MOTOR_FRAME_TVBS) {
        plane.nav_roll_cd = (int32_t)(tailsitter.tvbs_roll_gain * (float)plane.nav_roll_cd);
    }
    plane.nav_pitch_cd = wp_nav->get_pitch();
    
    // climb based on altitude error
    pos_control->set_alt_target_from_climb_rate_ff(assist_climb_rate_cms(), plane.G_Dt, false);
    run_z_controller();
}


/*
  handle auto-mode when auto_state.vtol_mode is true
 */
void QuadPlane::control_auto(const Location &loc)
{
    if (!setup()) {
        return;
    }

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    uint16_t id = plane.mission.get_current_nav_cmd().id;
    switch (id) {
    case MAV_CMD_NAV_VTOL_TAKEOFF:
    case MAV_CMD_NAV_TAKEOFF:
        if (is_vtol_takeoff(id)) {
            takeoff_controller();
        }
        break;
    case MAV_CMD_NAV_VTOL_LAND:
    case MAV_CMD_NAV_LAND:
        if (is_vtol_land(id)) {
            vtol_position_controller();
        }
        break;
    case MAV_CMD_NAV_LOITER_UNLIM:
    case MAV_CMD_NAV_LOITER_TIME:
    case MAV_CMD_NAV_LOITER_TURNS:
    case MAV_CMD_NAV_LOITER_TO_ALT:
        vtol_position_controller();
        break;
    default:
        waypoint_controller();
        break;
    }
}

/*
  handle QRTL mode
 */
void QuadPlane::control_qrtl(void)
{
    vtol_position_controller();
    if (poscontrol.state >= QPOS_POSITION2) {
        // change target altitude to home alt
        plane.next_WP_loc.alt = plane.home.alt;
        verify_vtol_land();
    }
}

/*
  handle LAND mode
 */
void QuadPlane::control_qland(void)
{
    vtol_position_controller();
    if (poscontrol.state >= QPOS_POSITION2) {
        verify_vtol_land();
    }
}

/*
  handle QRTL mode
 */
void QuadPlane::init_qrtl(void)
{
    // use do_RTL() to setup next_WP_loc
    plane.do_RTL(plane.home.alt + qrtl_alt*100UL);
    plane.prev_WP_loc = plane.current_loc;
    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
    poscontrol.state = QPOS_POSITION1;
    poscontrol.initialised = false;
    pos_control->set_desired_accel_xy(0.0f, 0.0f);
    pos_control->init_xy_controller();
}

/*
  handle LAND mode
 */
void QuadPlane::init_qland(void)
{
    plane.prev_WP_loc = plane.current_loc;
    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
    poscontrol.state = QPOS_POSITION1;
    poscontrol.initialised = false;
    pos_control->set_desired_accel_xy(0.0f, 0.0f);
    pos_control->init_xy_controller();
}

/*
  start a VTOL takeoff
 */
bool QuadPlane::do_vtol_takeoff(const AP_Mission::Mission_Command& cmd)
{
    if (!setup()) {
        return false;
    }

    plane.set_next_WP(cmd.content.location);
    if (options & OPTION_RESPECT_TAKEOFF_FRAME) {
        if (plane.current_loc.alt >= plane.next_WP_loc.alt) {
            // we are above the takeoff already, no need to do anything
            return false;
        }
    } else {
        plane.next_WP_loc.alt = plane.current_loc.alt + cmd.content.location.alt;
    }
    throttle_wait = false;
    takeoff_reset_complete = false;
    init_takeoff_this_frame = false;

    // set target to current position
    loiter_nav->clear_pilot_desired_acceleration();
    loiter_nav->init_target();

    // initialize vertical speed and acceleration
    pos_control->set_max_speed_z(-pilot_velocity_z_max, pilot_velocity_z_max);
    pos_control->set_max_accel_z(pilot_accel_z);

    // initialise position and desired velocity
    set_alt_target_current();
    takeoff_pos_cm = inertial_nav.get_position();
    pos_control->set_xy_target(takeoff_pos_cm.x, takeoff_pos_cm.y);
    pos_control->set_desired_velocity_xy(0.0f,0.0f);
    pos_control->set_desired_accel_xy(0.0f,0.0f);
    takeoff_alt_cm = plane.current_loc.alt;
    pos_control->set_desired_velocity_z(0.0f);

    // tell position controller to prepare for takeoff
    pos_control->init_takeoff();

    // also update nav_controller for status output
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    return true;
}


/*
  start a VTOL landing
 */
bool QuadPlane::do_vtol_land(const AP_Mission::Mission_Command& cmd)
{
    if (!setup()) {
        return false;
    }
    attitude_control->reset_rate_controller_I_terms();
    pos_control->get_accel_z_pid().reset_I();
    pos_control->get_vel_xy_pid().reset_I();
    
    plane.set_next_WP(cmd.content.location);
    // initially aim for current altitude
    plane.next_WP_loc.alt = plane.current_loc.alt;
    poscontrol.state = QPOS_POSITION1;
    poscontrol.initialised = false;
    pos_control->set_desired_accel_xy(0.0f, 0.0f);
    pos_control->init_xy_controller();

    throttle_wait = false;
    landing_detect.lower_limit_start_ms = 0;
    set_alt_target_current();

    plane.crash_state.is_crashed = false;
    
    // also update nav_controller for status output
    plane.nav_controller->update_waypoint(plane.prev_WP_loc, plane.next_WP_loc);
    return true;
}

/*
  check if a VTOL takeoff has completed
 */
bool QuadPlane::verify_vtol_takeoff(const AP_Mission::Mission_Command &cmd)
{
    if (!available()) {
        return true;
    }
    if (plane.current_loc.alt < plane.next_WP_loc.alt) {
        return false;
    }
    transition_state = is_tailsitter() ? TRANSITION_ANGLE_WAIT_FW : TRANSITION_AIRSPEED_WAIT;
    plane.TECS_controller.set_pitch_max_limit(transition_pitch_max);
    set_alt_target_current();

    plane.complete_auto_takeoff();
    
    return true;
}

/*
  check if a landing is complete
 */
void QuadPlane::check_land_complete(void)
{
    if (poscontrol.state != QPOS_LAND_FINAL) {
        // only apply to final landing phase
        return;
    }
    const uint32_t now = AP_HAL::millis();
    bool might_be_landed =  (landing_detect.lower_limit_start_ms != 0 &&
                             now - landing_detect.lower_limit_start_ms > 1000);
    if (!might_be_landed) {
        landing_detect.land_start_ms = 0;
        return;
    }
    float height = inertial_nav.get_altitude()*0.01f;
    if (landing_detect.land_start_ms == 0) {
        landing_detect.land_start_ms = now;
        landing_detect.vpos_start_m = height;
    }
    // we only consider the vehicle landed when the motors have been
    // at minimum for 5s and the vertical position estimate has not
    // changed by more than 20cm for 4s
    if (fabsf(height - landing_detect.vpos_start_m) > 0.2) {
        // height has changed, call off landing detection
        landing_detect.land_start_ms = 0;
        return;
    }
           
    if ((now - landing_detect.land_start_ms) < 4000 ||
        (now - landing_detect.lower_limit_start_ms) < 5000) {
        // not landed yet
        return;
    }
    landing_detect.land_start_ms = 0;
    // motors have been at zero for 5s, and we have had less than 0.3m
    // change in altitude for last 4s. We are landed.
    plane.disarm_motors();
    poscontrol.state = QPOS_LAND_COMPLETE;
    gcs().send_text(MAV_SEVERITY_INFO,"Land complete");
    // reload target airspeed which could have been modified by the mission
    plane.aparm.airspeed_cruise_cm.load();
}

/*
  check if a VTOL landing has completed
 */
bool QuadPlane::verify_vtol_land(void)
{
    if (!available()) {
        return true;
    }
    if (poscontrol.state == QPOS_POSITION2 &&
        !poscontrol.outside_cone) {
        poscontrol.state = QPOS_LAND_DESCEND;
        gcs().send_text(MAV_SEVERITY_INFO,"Land descend started");
        plane.set_next_WP(plane.next_WP_loc);
        _auto_land_arrested = false;
        _land_point_offset_NE.zero();
        descent_delay_time_sec = 0.0f;
    }

    if (should_relax()) {
        loiter_nav->soften_for_landing();
    }

    // at land_final_alt begin final landing
    if (poscontrol.state == QPOS_LAND_DESCEND && _height_above_ground_m < land_final_alt) {
        poscontrol.state = QPOS_LAND_FINAL;
        descent_delay_time_sec = 0.0f;

        // cut IC engine if enabled
        if (land_icengine_cut != 0) {
            plane.g2.ice_control.engine_control(0, 0, 0);
        }
        gcs().send_text(MAV_SEVERITY_INFO,"Land final started");
    }

    check_land_complete();
    return false;
}

// Write a control tuning packet
void QuadPlane::Log_Write_QControl_Tuning()
{
    const Vector3f &desired_velocity = pos_control->get_desired_velocity();
    const Vector3f &accel_target = pos_control->get_accel_target();
    struct log_QControl_Tuning pkt = {
        LOG_PACKET_HEADER_INIT(LOG_QTUN_MSG),
        time_us             : AP_HAL::micros64(),
        angle_boost         : attitude_control->angle_boost(),
        throttle_out        : motors->get_throttle(),
        desired_alt         : pos_control->get_alt_target() / 100.0f,
        inav_alt            : inertial_nav.get_altitude() / 100.0f,
        desired_climb_rate  : (int16_t)pos_control->get_vel_target_z(),
        climb_rate          : (int16_t)inertial_nav.get_velocity_z(),
        dvx                 : desired_velocity.x*0.01f,
        dvy                 : desired_velocity.y*0.01f,
        dax                 : accel_target.x*0.01f,
        day                 : accel_target.y*0.01f,
        throttle_mix        : attitude_control->get_throttle_mix(),
        tvbs_gain_mod       : _limit_cycle_gain_modifier,
    };
    plane.DataFlash.WriteBlock(&pkt, sizeof(pkt));

}

void QuadPlane::Log_Write_MotBatt()
{
    struct log_MotBatt pkt_mot = {
        LOG_PACKET_HEADER_INIT(LOG_MOTBATT_MSG),
        time_us         : AP_HAL::micros64(),
        lift_max        : (float)(motors->get_lift_max()),
        bat_volt        : (float)(motors->get_batt_voltage_filt()),
        fw_th_gain      : (float)_fw_throttle_factor,
        th_limit        : (float)(motors->get_throttle_limit())
    };
    plane.DataFlash.WriteBlock(&pkt_mot, sizeof(pkt_mot));
}

/*
  calculate the forward throttle percentage. The forward throttle can
  be used to assist with position hold and with landing approach. It
  reduces the need for down pitch which reduces load on the vertical
  lift motors.
 */
int8_t QuadPlane::forward_throttle_pct(void)
{
    /*
      in non-VTOL modes or modes without a velocity controller. We
      don't use it in QHOVER or QSTABILIZE as they are the primary
      recovery modes for a quadplane and need to be as simple as
      possible. They will drift with the wind
    */
    if (!in_vtol_mode() ||
        !motors->armed() ||
        vel_forward.gain <= 0 ||
        plane.control_mode == QSTABILIZE ||
        plane.control_mode == QHOVER) {
        return 0;
    }

    float deltat = (AP_HAL::millis() - vel_forward.last_ms) * 0.001f;
    if (deltat > 1 || deltat < 0) {
        vel_forward.integrator = 0;
        deltat = 0.1;
    }
    if (deltat < 0.1) {
        // run at 10Hz
        return vel_forward.last_pct;
    }
    vel_forward.last_ms = AP_HAL::millis();
    
    // work out the desired speed in forward direction
    const Vector3f &desired_velocity_cms = pos_control->get_desired_velocity();
    Vector3f vel_ned;
    if (!plane.ahrs.get_velocity_NED(vel_ned)) {
        // we don't know our velocity? EKF must be pretty sick
        vel_forward.last_pct = 0;
        vel_forward.integrator = 0;
        return 0;
    }
    Vector3f vel_error_body = ahrs.get_rotation_body_to_ned().transposed() * ((desired_velocity_cms*0.01f) - vel_ned);

    // find component of velocity error in fwd body frame direction
    float fwd_vel_error = vel_error_body * Vector3f(1,0,0);

    // scale forward velocity error by maximum airspeed
    fwd_vel_error /= MAX(plane.aparm.airspeed_max, 5);

    // add in a component from our current pitch demand. This tends to
    // move us to zero pitch. Assume that LIM_PITCH would give us the
    // WP nav speed.
    fwd_vel_error -= (wp_nav->get_speed_xy() * 0.01f) * plane.nav_pitch_cd / (float)plane.aparm.pitch_limit_max_cd;

    if (should_relax() && vel_ned.length() < 1) {
        // we may be landed
        fwd_vel_error = 0;
        vel_forward.integrator *= 0.95f;
    }
    
    // integrator as throttle percentage (-100 to 100)
    vel_forward.integrator += fwd_vel_error * deltat * vel_forward.gain * 100;

    // inhibit reverse throttle and allow petrol engines with min > 0
    int8_t fwd_throttle_min = plane.have_reverse_thrust() ? 0 : plane.aparm.throttle_min;
    vel_forward.integrator = constrain_float(vel_forward.integrator, fwd_throttle_min, plane.aparm.throttle_max);
    
    // If we are below alt_cutoff then scale down the effect until it turns off at alt_cutoff and decay the integrator
    float alt_cutoff = MAX(0,vel_forward_alt_cutoff);
    vel_forward.last_pct = linear_interpolate(0, vel_forward.integrator,
                                   _height_above_ground_m, alt_cutoff, alt_cutoff+2);
    if (vel_forward.last_pct == 0) {
        // if the percent is 0 then decay the integrator
        vel_forward.integrator *= 0.95f;
    }

    return vel_forward.last_pct;
}

/*
  get weathervaning yaw rate in cd/s
 */
float QuadPlane::get_weathervane_yaw_rate_cds(void)
{
    float delta_time = 0.001f * (float)(AP_HAL::millis() - weathervane.last_frame_ms);
    weathervane.last_frame_ms = AP_HAL::millis();

    /*
      we only do weathervaning in modes where we are doing VTOL
      position control and not during transition back into VTOL mode.
    */
    if (!in_vtol_mode() ||
        !motors->armed() ||
        (fabsf(weathervane.gain) < 0.01f) ||
        reverse_transition_active ||
        (plane.control_mode == QSTABILIZE) ||
        (plane.control_mode == QHOVER)) {
        weathervane.last_output = 0;
        return 0;
    }

    // check if we are flying backwards relative to the wind
    bool is_flying_backwards;
    if (weathervane.gain > 0.0f) {
        // the pilots is flying backwards which is opposite to the desired wind relative yaw
        is_flying_backwards = (plane.channel_pitch->get_control_in() > 0) && (AP::ins().get_accel().z > 1.0f);
    } else {
        // the pilots is flying forwards which is opposite to the desired wind relative yaw
        is_flying_backwards = (plane.channel_pitch->get_control_in() < 0) && (AP::ins().get_accel().z < -1.0f);

    }
    // determine if the pilot has input a yaw or roll input in the last 3 seconds
    // also check if the pilot is flying backwards relative to the desired wind relative yaw
    if (plane.channel_rudder->get_control_in() != 0
            || plane.channel_roll->get_control_in() != 0
            || is_flying_backwards) {
        weathervane.last_pilot_input_ms = AP_HAL::millis();
        weathervane.last_output = 0.0f;
    }

    // reduce the gain when the pilot is attempting to yaw, move sideways or move backwards
    float gain_modifier_input = 1.0f - constrain_float(fabsf(plane.channel_roll->get_control_in() / 4500.0f),0.0f,1.0f);
    gain_modifier_input *= 1.0f - constrain_float(fabsf(plane.channel_rudder->get_control_in() / 4500.0f),0.0f,1.0f);
    gain_modifier_input *= 1.0f - constrain_float((plane.channel_pitch->get_control_in() / 4500.0f),0.0f,1.0f);

    const float recovery_rate = 0.25f;
    if (AP_HAL::millis() - weathervane.last_pilot_input_ms > 3000) {
        float max_increment = delta_time * recovery_rate;
        gain_modifier_input = 1.0f;
        if (gain_modifier_input - weathervane.gain_modifier > max_increment) {
            weathervane.gain_modifier += max_increment;
        } else {
            weathervane.gain_modifier = gain_modifier_input;
        }
    } else {
        float max_increment = delta_time * recovery_rate * 0.1f;
        if (gain_modifier_input - weathervane.gain_modifier > max_increment) {
            weathervane.gain_modifier += max_increment;
        } else {
            weathervane.gain_modifier = gain_modifier_input;
        }
    }

    // check if we are moving backwards rel to ground
    Vector3f vel_ned;
    if (plane.ahrs.get_velocity_NED(vel_ned)) {
        float fwd_gnd_vel = vel_ned.x * plane.ahrs.cos_yaw() + vel_ned.y * plane.ahrs.sin_yaw();
        if (weathervane.moving_backwards && fwd_gnd_vel > -0.5f) {
            weathervane.moving_backwards = false;
        } else if (!weathervane.moving_backwards && fwd_gnd_vel < -1.0f) {
            weathervane.moving_backwards = true;
            weathervane.last_move_back_ms = weathervane.last_frame_ms;
        }
    } else {
        weathervane.moving_backwards = true;
        weathervane.last_move_back_ms = weathervane.last_frame_ms;
    }

    // Calculate a yaw rate proportional to lateral g when wind is from the front. When wind from the back use the
    // maximum of the lateral and longitudinal g with hysteresis to prevent getting stuck with back to wind
    float roll = wp_nav->get_roll() / 100.0f;
    float lateral_g = AP::ins().get_accel().y / GRAVITY_MSS;
    float rear_g = AP::ins().get_accel().z / GRAVITY_MSS;
    bool tailwind = rear_g > 0.05f;
    float error_g;
    if (tailwind && ((weathervane.last_frame_ms - weathervane.last_move_back_ms) > 3000)) {
        // hovering with a tailwind
        if (lateral_g * weathervane.last_output <= 0.0f) {
            // demanded yaw is consistent with lateral g so use
            if (lateral_g >= 0.0f)  {
                error_g = - MAX(lateral_g, rear_g);
            } else {
                error_g = MAX(-lateral_g, rear_g);
            }
        } else {
            // lateral g is inconsistent with previous demanded yaw, so continue yaw in same direction to avoid
            // indecision when back is facing directly into wind
            if (weathervane.last_output < 0.0f) {
                error_g = - rear_g;
            } else {
                error_g = rear_g;
            }
        }
        // integrate rate so that max yaw rate is achieved until facing wind even for small tail wind values
        weathervane.last_output += error_g * weathervane.gain * delta_time;
        weathervane.last_output = constrain_float(weathervane.last_output,-1.0f, 1.0f);
    } else {
        float output;
        if (should_relax()) {
            output = 0.0f;
        } else {
            output = constrain_float(-lateral_g * weathervane.gain, -1, 1);
        }
        weathervane.last_output = 0.98f * weathervane.last_output + 0.02f * output;
    }

    // check if there is  a landing tipover risk
    if (rear_g > 0.05f || weathervane.moving_backwards) {
        weathervane.tip_warning = true;
    } else {
        weathervane.tip_warning = false;
    }

    // when wind is from front, allow yaw demand to go to zero if bank angle is within specified limits
    if (fabsf(roll) < weathervane.min_roll && !tailwind) {
        weathervane.last_output = 0;
        return 0;
    }

    // If the wind is from the back or there is excessive lateral g indicating outside crosswind limits, then ignore payload yaw pointing
    float g_metric = MAX(fabsf(lateral_g), rear_g);
    if (!weathervane.payload_yaw_lockout && (g_metric > tailsitter.tvbs_lat_gmax)) {
        weathervane.payload_yaw_lockout = true;
    } else if (weathervane.payload_yaw_lockout && (g_metric < 0.5f * tailsitter.tvbs_lat_gmax)) {
        weathervane.payload_yaw_lockout = false;
        plane.camera_mount.set_yaw_target(degrees(ahrs_view->yaw));
    }

    float yaw_rate_dem_cd;
    if (plane.vtolCameraControlMode
            && (tailsitter.tvbs_yaw_gain > 0.1f)
            && !weathervane.payload_yaw_lockout) {
        // when flying in a VTOL mode where the operator is panning the payload mount, yaw the vehicle to drive vehicle relative yaw to zero
        // enables yaw pointing of payloads with limited angular motion, eg roll tilt gimbals
        yaw_rate_dem_cd = 100.0f * constrain_float(tailsitter.tvbs_yaw_gain * degrees(wrap_PI(plane.camera_mount.get_ef_yaw() - ahrs_view->yaw)), -yaw_rate_max, yaw_rate_max);
    } else {
        // do normal weather vaning over half of yaw_rate_max. This gives the pilot twice the
        // authority of the weathervane controller
        yaw_rate_dem_cd = weathervane.gain_modifier * weathervane.last_output * (yaw_rate_max/2) * 100;
    }
    return yaw_rate_dem_cd;

}

/*
  start guided mode control
 */
void QuadPlane::guided_start(void)
{
    poscontrol.state = QPOS_POSITION1;
    poscontrol.initialised = false;
    guided_takeoff = false;
    _auto_land_arrested = false;
    _land_point_offset_NE.zero();
    setup_target_position();
    poscontrol.slow_descent = (plane.current_loc.alt > plane.next_WP_loc.alt);
}

/*
  update guided mode control
 */
void QuadPlane::guided_update(void)
{
    if (plane.control_mode == GUIDED && guided_takeoff && plane.current_loc.alt < plane.next_WP_loc.alt) {
        throttle_wait = false;
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        takeoff_controller();
    } else {
        guided_takeoff = false;
        // run VTOL position controller
        vtol_position_controller();
    }
}

void QuadPlane::afs_terminate(void)
{
    if (available()) {
        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        motors->output();
    }
}

/*
  return true if we should do guided mode loitering using VTOL motors
 */
bool QuadPlane::guided_mode_enabled(void)
{
    if (!available()) {
        return false;
    }
    // only use quadplane guided when in AUTO or GUIDED mode
    if (plane.control_mode != GUIDED && plane.control_mode != AUTO) {
        return false;
    }
    return guided_mode != 0;
}

/*
  set altitude target to current altitude
 */
void QuadPlane::set_alt_target_current(void)
{
    pos_control->set_alt_target(inertial_nav.get_altitude());
}

/*
  adjust the altitude target to the given target, moving it slowly
 */
void QuadPlane::adjust_alt_target(float altitude_cm)
{
    float current_alt = inertial_nav.get_altitude();
    // don't let it get beyond 50cm from current altitude
    float target_cm = constrain_float(altitude_cm, current_alt-50, current_alt+50);
    pos_control->set_alt_target(target_cm);
}

// user initiated takeoff for guided mode
bool QuadPlane::do_user_takeoff(float takeoff_altitude)
{
    if (plane.control_mode != GUIDED) {
        gcs().send_text(MAV_SEVERITY_INFO, "User Takeoff only in GUIDED mode");
        return false;
    }
    if (!hal.util->get_soft_armed()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Must be armed for takeoff");
        return false;
    }
    if (is_flying()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Already flying - no takeoff");
        return false;
    }
    plane.auto_state.vtol_loiter = true;
    plane.prev_WP_loc = plane.current_loc;
    plane.next_WP_loc = plane.current_loc;
    plane.next_WP_loc.alt += takeoff_altitude*100;
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    guided_start();
    guided_takeoff = true;
    return true;
}

// return true if the wp_nav controller is being updated
bool QuadPlane::using_wp_nav(void) const
{
    return plane.control_mode == QLOITER || plane.control_mode == QLAND || plane.control_mode == QRTL;
}

/*
  return mav_type for heartbeat
 */
MAV_TYPE QuadPlane::get_mav_type(void) const
{
    if (mav_type.get() == 0) {
        return MAV_TYPE_FIXED_WING;
    }
    return MAV_TYPE(mav_type.get());
}

/*
  return true if current mission item is a vtol takeoff
*/
bool QuadPlane::is_vtol_takeoff(uint16_t id) const
{
    if (id == MAV_CMD_NAV_VTOL_TAKEOFF) {
        return true;
    }
    if (id == MAV_CMD_NAV_TAKEOFF && available() && (options & OPTION_ALLOW_FW_TAKEOFF) == 0) {
        // treat fixed wing takeoff as VTOL takeoff
        return true;
    }
    return false;
}

/*
  return true if current mission item is a vtol land
*/
bool QuadPlane::is_vtol_land(uint16_t id) const
{
    if (id == MAV_CMD_NAV_VTOL_LAND) {
        if (options & QuadPlane::OPTION_MISSION_LAND_FW_APPROACH) {
            return plane.vtol_approach_s.approach_stage == Plane::Landing_ApproachStage::VTOL_LANDING;
        } else {
            return true;
        }
    }
    if (id == MAV_CMD_NAV_LAND && available() && (options & OPTION_ALLOW_FW_LAND) == 0) {
        // treat fixed wing land as VTOL land
        return true;
    }
    return false;
}

/*
  return true if we are in a transition to fwd flight from hover
 */
bool QuadPlane::in_transition(void) const
{
    return available() && assisted_flight &&
        (transition_state == TRANSITION_AIRSPEED_WAIT ||
         transition_state == TRANSITION_TIMER);
}

/*
  calculate current stopping distance for a quadplane in fixed wing flight
 */
float QuadPlane::stopping_distance(void)
{
    // use v^2/(2*accel). This is only quite approximate as the drag
    // varies with pitch, but it gives something for the user to
    // control the transition distance in a reasonable way
    return plane.ahrs.groundspeed_vector().length_squared() / (2 * transition_decel);
}
