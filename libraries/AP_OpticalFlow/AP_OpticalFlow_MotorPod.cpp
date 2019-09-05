/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
  driver for MotorPod Pixart PAW3903E1 optical flow sensor
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include "OpticalFlow.h"
#include "AP_OpticalFlow_MotorPod.h"
#include <stdio.h>

#define debug(fmt, args ...)  do {printf(fmt, ## args); } while(0)


AP_OpticalFlow_MotorPod::AP_OpticalFlow_MotorPod(OpticalFlow &_frontend) :
    OpticalFlow_backend(_frontend)
{
    // empty
}


// detect the device
AP_OpticalFlow_MotorPod *AP_OpticalFlow_MotorPod::detect(OpticalFlow &_frontend) {
    if (AP::motorPod() != nullptr) {
        AP_OpticalFlow_MotorPod *sensor = new AP_OpticalFlow_MotorPod(_frontend);
        if (sensor) {
            return sensor;
        }
    }
    return nullptr;
}

void AP_OpticalFlow_MotorPod::update(void) {
    if (AP::motorPod() != nullptr) {
        return;
    }

    WITH_SEMAPHORE(_sem);

    uint32_t now_us = AP_HAL::micros();
    // TODO: configure this properly
    if ((now_us - lastUpdate_us) < 100000) {
        return;
    }

    // Get optical flow data from MotorPod driver
    AP_PpdsMotorPod::FlowData flowData;
    bool gotData = AP::motorPod()->getFlowData(&flowData);

    // return without updating state if no readings
    if (!gotData) {
        return;
    }

    const float kMicrosToSeconds = 1.0e-6;
    float dt_gyro = (now_us - lastUpdate_us) * kMicrosToSeconds;
    lastUpdate_us = now_us;

    struct OpticalFlow::OpticalFlow_state state;
    state.surface_quality = flowData.surfaceQuality;

    const Vector3f& gyro = AP::ahrs_navekf().get_gyro();
    state.bodyRate = Vector2f(gyro.x / dt_gyro, gyro.y / dt_gyro);

    const float dt_flow = flowData.delta.t_us * kMicrosToSeconds;
    const float kTimeout = 0.3; // TODO: configure this
    if (is_positive(dt_flow) && (dt_flow < kTimeout)) {

        const Vector2f flowScaler = _flowScaler();
        float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
        float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;

        state.flowRate = Vector2f(flowData.delta.x * flowScaleFactorX,
                                  flowData.delta.y * flowScaleFactorY);
        state.flowRate *= kFlowPixelScaling / dt_flow;

        // we only apply yaw to flowRate as body rate comes from AHRS
        _applyYaw(state.flowRate);

        AP::motorPod()->clearFlowData();
    } else {
        state.flowRate.zero();
    }

    // copy results to front end
    _update_frontend(state);
}
