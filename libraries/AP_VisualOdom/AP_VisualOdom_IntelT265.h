#pragma once

#include "AP_VisualOdom_config.h"

#if AP_VISUALODOM_INTELT265_ENABLED

#include "AP_VisualOdom_Backend.h"

class AP_VisualOdom_IntelT265 : public AP_VisualOdom_Backend
{

public:

    using AP_VisualOdom_Backend::AP_VisualOdom_Backend;

    // consume vision pose estimate data and send to EKF. distances in meters
    // quality of -1 means failed, 0 means unknown, 1 is worst, 100 is best
    void handle_pose_estimate(uint64_t remote_time_us, uint32_t time_ms, float x, float y, float z, const Quaternion &attitude, float posErr, float angErr, uint8_t reset_counter, int8_t quality) override;

    // consume vision velocity estimate data and send to EKF, velocity in NED meters per second
    // quality of -1 means failed, 0 means unknown, 1 is worst, 100 is best
    void handle_vision_speed_estimate(uint64_t remote_time_us, uint32_t time_ms, const Vector3f &vel, uint8_t reset_counter, int8_t quality) override;

    // request sensor's yaw be aligned with vehicle's AHRS/EKF attitude
    void request_align_yaw_to_ahrs() override { _align_yaw = true; }

    // arming check
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;

protected:

    // apply rotation to velocity
    void rotate_velocity(Vector3f &velocity) const;

    // rotate attitude using _yaw_trim
    void rotate_attitude(Quaternion &attitude) const;

    // use sensor provided position and attitude to calculate rotation to align sensor yaw with AHRS/EKF attitude
    // calls align_yaw (see below)
    bool align_yaw_to_ahrs(const Vector3f &position, const Quaternion &attitude);

    // align sensor yaw with any new yaw (in radians)
    void align_yaw(const Vector3f &position, const Quaternion &attitude, float yaw_rad);

    // returns true if sensor data should be consumed, false if it should be ignored
    // set vision_position_estimate to true if reset_counter is from the VISION_POSITION_ESTIMATE source, false otherwise
    // only the VISION_POSITION_ESTIMATE message's reset_counter is used to determine if sensor data should be ignored
    bool should_consume_sensor_data(bool vision_position_estimate, uint8_t reset_counter);

    // record voxl camera's position and reset counter for reset jump handling
    // position is post scaling, offset and orientation corrections
    void record_voxl_position_and_reset_count(const Vector3f &position, uint8_t reset_counter);

    // handle voxl camera reset jumps in attitude and position
    // sensor_pos should be the position directly from the sensor with only scaling applied (i.e. no yaw or position corrections)
    // sensor_att is similarly the attitude directly from the sensor
    void handle_voxl_camera_reset_jump(const Vector3f &sensor_pos, const Quaternion &sensor_att, uint8_t reset_counter);

    float _yaw_trim;                            // yaw angle trim (in radians) to align camera's yaw to ahrs/EKF's
    Quaternion _yaw_rotation;                   // earth-frame yaw rotation to align heading of sensor with vehicle.  use when _yaw_trim is non-zero
    Quaternion _att_rotation;                   // body-frame rotation corresponding to ORIENT parameter.  use when get_orientation != NONE
    bool _use_att_rotation;                     // true if _att_rotation should be applied to sensor's attitude data
    bool _align_yaw = true;                     // true if sensor yaw should be aligned to AHRS/EKF
    bool _error_orientation;                    // true if the orientation is not supported
    Quaternion _attitude_last;                  // last attitude received from camera (used for arming checks)
    uint8_t _pos_reset_counter_last;            // last vision-position-estimate reset counter value
    uint32_t _pos_reset_ignore_start_ms;        // system time we start ignoring sensor information, 0 if sensor data is not being ignored

    // voxl reset jump handling variables
    uint8_t _voxl_reset_counter_last;           // last reset counter from voxl camera (only used for origin jump handling)
    Vector3f _voxl_position_last;               // last recorded position (post scaling, offset and orientation corrections)
};

#endif  // AP_VISUALODOM_INTELT265_ENABLED
