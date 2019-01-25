#pragma once

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_Common.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <DataFlash/DataFlash.h>
#include "AP_AutoTune.h"
#include <cmath>

class AP_YawController {
public:
    AP_YawController(AP_AHRS &ahrs, const AP_Vehicle::FixedWing &parms, DataFlash_Class &_dataflash)
        : aparm(parms)
        , autotune(gains, AP_AutoTune::AUTOTUNE_ROLL, parms, _dataflash)
        , _ahrs(ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_YawController(const AP_YawController &other) = delete;
    AP_YawController &operator=(const AP_YawController&) = delete;

	int32_t get_servo_out(float scaler, bool disable_integrator);

	void reset_I();

	const DataFlash_Class::PID_Info& get_pid_info(void) const {return _pid_info; }

	static const struct AP_Param::GroupInfo var_info[];

    // tuning accessors
    void kP(float v) { gains.P.set(v); }
    void kI(float v) { gains.I.set(v); }
    void kD(float v) { gains.D.set(v); }
    void kFF(float v) { gains.FF.set(v); }

    AP_Float &kP(void) { return gains.P; }
    AP_Float &kI(void) { return gains.I; }
    AP_Float &kD(void) { return gains.D; }
    AP_Float &kFF(void) { return gains.FF; }

private:
    const AP_Vehicle::FixedWing &aparm;
    AP_AutoTune::ATGains gains;
    AP_AutoTune autotune;
    AP_Int16 _imax;
    AP_Float _tau;
    uint32_t _last_t;
	float _last_out;
	float _last_rate_hp_out;
	float _last_rate_hp_in;
	float _K_D_last;

	float _integrator;

	DataFlash_Class::PID_Info _pid_info;

	AP_AHRS &_ahrs;
};
