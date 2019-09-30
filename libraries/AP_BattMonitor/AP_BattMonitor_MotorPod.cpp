#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_MotorPod.h"
#include "AP_PpdsMotorPod/AP_PpdsMotorPod.hpp"

extern const AP_HAL::HAL& hal;


// TODO: this is a bit of a hack. Is there a better way?
// For this driver to work, the AP_PpdsMotorPod driver needs to be included for
// compilation. As we don't have feature flags or other conditional compilation,
// we add weak versions of the required AP_PpdsMotorPod functions to allow this
// to compile without the AP_PpdsMotorPod driver.
namespace AP {
    WEAK AP_PpdsMotorPod *motorPod() {
        return nullptr;
    }
}
WEAK bool AP_PpdsMotorPod::getAdcData(AdcState_t * const adc_data) { return false; }


/// Constructor
AP_BattMonitor_MotorPod::AP_BattMonitor_MotorPod(AP_BattMonitor &mon,
                                                 AP_BattMonitor::BattMonitor_State &mon_state,
                                                 AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{

}

// read - read the voltage and current
void AP_BattMonitor_MotorPod::read() {
    if (AP::motorPod() == nullptr) {
        _state.healthy = false;
        return;
    }

    AdcState_t adcData;
    bool gotData = AP::motorPod()->getAdcData(&adcData);

    // return without updating state if no readings
    if (!gotData) {
        _state.healthy = false;
        return;
    }

    _state.healthy = true;

    _state.current_amps = adcData.current;
    _state.voltage = adcData.voltage * _params._volt_multiplier;
    _state.temperature = adcData.temperature;

    _state.temperature_time = AP_HAL::millis();
}
