#include "AP_Mount_Servo.h"

extern const AP_HAL::HAL& hal;

// init - performs any required initialisation for this instance
void AP_Mount_Servo::init(const AP_SerialManager& serial_manager)
{
    if (_instance == 0) {
        _roll_idx = SRV_Channel::k_mount_roll;
        _tilt_idx = SRV_Channel::k_mount_tilt;
        _pan_idx  = SRV_Channel::k_mount_pan;
        _open_idx = SRV_Channel::k_mount_open;
    } else {
        // this must be the 2nd mount
        _roll_idx = SRV_Channel::k_mount2_roll;
        _tilt_idx = SRV_Channel::k_mount2_tilt;
        _pan_idx  = SRV_Channel::k_mount2_pan;
        _open_idx = SRV_Channel::k_mount2_open;
    }

    // check which servos have been assigned
    check_servo_map();
}

// update mount position - should be called periodically
void AP_Mount_Servo::update()
{
    static bool mount_open = 0;     // 0 is closed

    // check servo map every three seconds to allow users to modify parameters
    uint32_t now = AP_HAL::millis();
    if (now - _last_check_servo_map_ms > 3000) {
        check_servo_map();
        _last_check_servo_map_ms = now;
    }

    switch(get_mode()) {
        // move mount to a "retracted position" or to a position where a fourth servo can retract the entire mount into the fuselage
        case MAV_MOUNT_MODE_RETRACT:
        {
            _angle_bf_output_deg = _state._retract_angles.get();
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL:
        {
            _angle_bf_output_deg = _state._neutral_angles.get();
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
        {
            // earth-frame angle targets (i.e. _angle_ef_target_rad) should have already been set by a MOUNT_CONTROL message from GCS
            stabilize();
            break;
        }

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING:
        {
            // update targets using pilot's rc inputs
            if (!_slave_yaw_roll) {
                update_targets_from_rc();
            }
            stabilize();
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT:
        {
            if(AP::gps().status() >= AP_GPS::GPS_OK_FIX_2D) {
                calc_angle_to_location(_state._roi_target, _angle_ef_target_rad, true, true, false);
                stabilize();
            }
            break;
        }

        default:
            //do nothing
            break;
    }

    // move mount to a "retracted position" into the fuselage with a fourth servo
    bool mount_open_new = (get_mode() == MAV_MOUNT_MODE_RETRACT) ? 0 : 1;
    if (mount_open != mount_open_new) {
        mount_open = mount_open_new;
        move_servo(_open_idx, mount_open_new, 0, 1);
    }

    // write the results to the servos
    move_servo(_roll_idx, _angle_bf_output_deg.x*10, _state._roll_angle_min*0.1f, _state._roll_angle_max*0.1f);
    move_servo(_tilt_idx, _angle_bf_output_deg.y*10, _state._tilt_angle_min*0.1f, _state._tilt_angle_max*0.1f);
    move_servo(_pan_idx,  _angle_bf_output_deg.z*10, _state._pan_angle_min*0.1f, _state._pan_angle_max*0.1f);
}

// set_mode - sets mount's mode
void AP_Mount_Servo::set_mode(enum MAV_MOUNT_MODE mode)
{
    // record the mode change and return success
    _state._mode = mode;
}

// private methods

// check_servo_map - detects which axis we control using the functions assigned to the servos in the RC_Channel_aux
//  should be called periodically (i.e. 1hz or less)
void AP_Mount_Servo::check_servo_map()
{
    _flags.roll_control = SRV_Channels::function_assigned(_roll_idx);
    _flags.tilt_control = SRV_Channels::function_assigned(_tilt_idx);
    _flags.pan_control = SRV_Channels::function_assigned(_pan_idx);
}

// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void AP_Mount_Servo::send_mount_status(mavlink_channel_t chan)
{
    mavlink_msg_mount_status_send(chan, 0, 0, _angle_bf_output_deg.y*100, _angle_bf_output_deg.x*100, _angle_bf_output_deg.z*100);
}

// stabilize - stabilizes the mount relative to the Earth's frame
//  input: _angle_ef_target_rad (earth frame targets in radians)
//  output: _angle_bf_output_deg (body frame angles in degrees)
void AP_Mount_Servo::stabilize()
{
    /*
     * Customised to operate with a inner pitch and outer roll drive gimbal with two modes:
     * a) pointing along a designated LOS vector defined by _angle_ef_target_rad.z (pan) and _angle_ef_target_rad.y (tilt), or
     * b) maintaining a zero _angle_ef_target_rad.x (level horizon) and specified _angle_ef_target_rad.y (tilt).
     */

    AP_AHRS &ahrs = AP::ahrs();
    float gimbal_pitch_rad; // Inner gimbal pich rotation (rad). When zero, camera points along X body axis. When positive, camera points up.
    float gimbal_roll_rad; // Outer gimbal roll rotation (rad). When zero the camera is level with the wing. When positive, camera rolls right.
    if (_slave_yaw_roll) {
        // Do option b - keep camera level and at the specified pitch angle
        if (fabs(ahrs.roll) < M_PI_2) {
            gimbal_roll_rad = -ahrs.roll;
        } else if (ahrs.roll >= M_PI_2) {
            gimbal_roll_rad = - M_PI + ahrs.roll;
        } else if (ahrs.roll <= -M_PI_2) {
            gimbal_roll_rad = M_PI + ahrs.roll;
        } else {
            gimbal_roll_rad = -ahrs.roll;
        }
        gimbal_pitch_rad = _angle_ef_target_rad.y - ahrs.pitch;
        // fade roll angle to zero as pitch approaches +-90 degrees
        if (fabsf(ahrs.pitch) > radians(45.0f)) {
            float gain = 1.0f - ((fabsf(ahrs.pitch) - radians(45.0f)) / radians(45.0f));
            gimbal_roll_rad *= gain;
        }

    } else {
        // Do option a - calculate gimbal roll and pitch angle required to point along vector defined by the
        // 32 Tait-Bryan sequence of _angle_ef_target_rad.z  and _angle_ef_target_rad.y

        // calculate the LOS unit vector and rotate into body frame
        float h_length = cosf(_angle_ef_target_rad.y);
        Vector3f los_vec_ef = {h_length * cosf(_angle_ef_target_rad.z) , h_length * sinf(_angle_ef_target_rad.z) , -sinf(_angle_ef_target_rad.y)};
        Matrix3f Tbn = ahrs.get_rotation_body_to_ned();
        Tbn.transpose();
        Vector3f los_vec_bf = Tbn * los_vec_ef;

        // Calculate the 12 Tait Bryan sequence angles that align the X axis in the gimbal frame with los_vec_bf
        gimbal_pitch_rad = atan2f(sqrtf(los_vec_bf.y*los_vec_bf.y + los_vec_bf.z*los_vec_bf.z) , los_vec_bf.x);
        if (los_vec_bf.z > 0.0f) {
            gimbal_pitch_rad = -gimbal_pitch_rad;
            gimbal_roll_rad = atan2f(-los_vec_bf.y, los_vec_bf.z);
        } else {
            gimbal_roll_rad = atan2f(los_vec_bf.y, -los_vec_bf.z);
        }

    }

    // compensate for vehicle rates sensing, computation and actuation lag
    const Vector3f &gyro = ahrs.get_gyro();
    gimbal_pitch_rad -= (gyro.y * cosf(gimbal_roll_rad) + gyro.z * sinf(gimbal_roll_rad)) * _state._pitch_stb_lead;
    gimbal_roll_rad -= gyro.x * _state._roll_stb_lead;

    // convert to output units
    _angle_bf_output_deg.x = degrees(gimbal_roll_rad);
    _angle_bf_output_deg.y = degrees(gimbal_pitch_rad);
    _angle_bf_output_deg.z = 0.0f;
}

// closest_limit - returns closest angle to 'angle' taking into account limits.  all angles are in degrees * 10
int16_t AP_Mount_Servo::closest_limit(int16_t angle, int16_t angle_min, int16_t angle_max)
{
    // Make sure the angle lies in the interval [-180 .. 180[ degrees
    while (angle < -1800) angle += 3600;
    while (angle >= 1800) angle -= 3600;

    // Make sure the angle limits lie in the interval [-180 .. 180[ degrees
    while (angle_min < -1800) angle_min += 3600;
    while (angle_min >= 1800) angle_min -= 3600;
    while (angle_max < -1800) angle_max += 3600;
    while (angle_max >= 1800) angle_max -= 3600;

    // If the angle is outside servo limits, saturate the angle to the closest limit
    // On a circle the closest angular position must be carefully calculated to account for wrap-around
    if ((angle < angle_min) && (angle > angle_max)) {
        // angle error if min limit is used
        int16_t err_min = angle_min - angle + (angle<angle_min ? 0 : 3600);     // add 360 degrees if on the "wrong side"
        // angle error if max limit is used
        int16_t err_max = angle - angle_max + (angle>angle_max ? 0 : 3600);     // add 360 degrees if on the "wrong side"
        angle = err_min<err_max ? angle_min : angle_max;
    }

    return angle;
}

// move_servo - moves servo with the given id to the specified angle.  all angles are in degrees * 10
void AP_Mount_Servo::move_servo(uint8_t function_idx, int16_t angle, int16_t angle_min, int16_t angle_max)
{
	// saturate to the closest angle limit if outside of [min max] angle interval
	int16_t servo_out = closest_limit(angle, angle_min, angle_max);
	SRV_Channels::move_servo((SRV_Channel::Aux_servo_function_t)function_idx, servo_out, angle_min, angle_max);
}
