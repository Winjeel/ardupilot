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
WEAK bool AP_PpdsMotorPod::getAdcData(AdcData * const adc_data) { return false; }
WEAK void AP_PpdsMotorPod::clearAdcData(void) { /* empty */ }


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

    AP_PpdsMotorPod::AdcData adcData;
    bool gotData = AP::motorPod()->getAdcData(&adcData);

    // return without updating state if no readings
    if (!gotData) {
        // The MotorPod updates faster than this driver, so we expect data every
        // time we update. One missed sample is probably just a CRC failure - any
        // more means there's a problem.
        uint8_t const kMaxMissedSamples = 2;
        if (++_missed_samples > kMaxMissedSamples) {
            _state.healthy = false;
        }
    } else {
        _missed_samples = 0;
        AP::motorPod()->clearAdcData();

        _state.healthy = true;

        _state.current_amps = (adcData.current * _params._curr_amp_per_volt) + _params._curr_amp_offset;
        _state.voltage = adcData.voltage * _params._volt_multiplier;
        _state.temperature = adcData.temperature;

        float const kAmpsToMilliAmps = 1000.0f;
        float milliAmps = ((adcData.consumedAmps * _params._curr_amp_per_volt) + _params._curr_amp_offset) * kAmpsToMilliAmps;
        float const kMicrosecondsToHours = (1.0f / (60.0f * 60.0f * 1000.0f * 1000.0f));
        float milliAmpHours = milliAmps * adcData.deltaT_us * kMicrosecondsToHours;

        _state.consumed_mah += milliAmpHours;

        // _state.voltage_resting_estimate will be the same as _state.voltage, as we don't have _state.resistance set
        _state.consumed_wh += (milliAmpHours / kAmpsToMilliAmps) * _state.voltage_resting_estimate;

        uint32_t current_time_micros = AP_HAL::micros();
        _state.last_time_micros = current_time_micros;
        float const kMicrosToMills = (1.0f / 1000.0f);
        _state.temperature_time = current_time_micros * kMicrosToMills;
    }
}
