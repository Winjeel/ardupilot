#pragma once

#include "OpticalFlow.h"
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_PpdsMotorPod/AP_PpdsMotorPod.hpp>

class AP_OpticalFlow_MotorPod : public OpticalFlow_backend
{
public:
    /// constructor
    AP_OpticalFlow_MotorPod(OpticalFlow &_frontend);

    // no initialisation required
    void init() override {
        // empty
    }

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void) override;

    // detect if the sensor is available
    static AP_OpticalFlow_MotorPod *detect(OpticalFlow &_frontend);

private:

    const float kFlowPixelScaling = 1.26e-3;

    struct GyroAccum {
        float x;
        float y;
        uint64_t t;
    } gyro_accum;

    uint32_t lastUpdate_us;
};
