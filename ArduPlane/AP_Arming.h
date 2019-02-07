#pragma once

#include <AP_Arming/AP_Arming.h>

/*
  a plane specific arming class
 */
class AP_Arming_Plane : public AP_Arming
{
public:
    AP_Arming_Plane()
        : AP_Arming()
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_Arming_Plane(const AP_Arming_Plane &other) = delete;
    AP_Arming_Plane &operator=(const AP_Arming_Plane&) = delete;

    bool pre_arm_checks(bool report);

    void check_shake_to_arm(void);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    bool ins_checks(bool report);

private:
    /*
      Used by Corvo X shake to start auto mission functonality
    */
    struct {
        uint32_t last_time_ms = 0; // last time the calculation was performed (msec)

        float ang_rot_x_hpf = 0.0f; // rotation about the X body axis after application of a high pass filter (rad)
        uint32_t ang_x_rh_time_ms = 0; // time a RH rotation was detected (msec)
        uint32_t ang_x_lh_time_ms = 0; // time a LH rotation was detected (msec)

        uint8_t up_shake_count = 0; // number of up shake events
        uint8_t down_shake_count = 0; // number of down shake events
        uint32_t up_shake_time_ms = 0; // last time an up shake event was detected (msec)
        uint32_t down_shake_time_ms = 0; // last time a down shake event was detected (msec)
        uint32_t first_shake_time_ms = 0; // time the first shake event, either up or down, was detected (msec)
        uint32_t shake_pass_time_ms = 0; // time the shake test passed (msec)
        float accel_up_filt = 0.0f; // state for LPF noise filter applied to vertical acceleration

    } shake_to_fly;
    uint32_t shake_arm_time_ms = 0; // time the vehicle was armed using  the 'shake to fly' method (msec)

};
