#pragma once

#include "BoardTestConstants.h"
#include <AP_Math/definitions.h> // GRAVITY_MSS
#include <vector>

bool _checkGravityAcceleration(float acceleration);
bool _checkRotation(float rotation);
float _calculateVectorAverage(std::vector<float> sampleWindow);

void _initialiseRandomNumberGenerator(void);
bool _flushUART(AP_HAL::UARTDriver* serialDevice);