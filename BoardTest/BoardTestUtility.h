#pragma once

#include "BoardTestConstants.h"
#include <AP_Math/definitions.h> // GRAVITY_MSS
#include <vector>

bool _checkGravityAcceleration(float acceleration);
bool _checkRotation(float rotation);
float _approxRunningAverage(float average, float newSample);

void _initialiseRandomNumberGenerator(void);