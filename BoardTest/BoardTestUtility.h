#pragma once

#include "BoardTestConstants.h"
#include <AP_Math/definitions.h>

bool _checkGravityAcceleration(float);
bool _checkRotation(float);
bool _checkCompassAlignment(float);
float _approxRunningAverage(float, float);