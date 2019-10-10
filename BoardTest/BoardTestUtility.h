#pragma once

#include "BoardTestConstants.h"
#include <AP_Math/definitions.h> // GRAVITY_MSS
#include <vector>

bool _checkGravityAcceleration(float);
bool _checkRotation(float);
bool _checkCompassAlignment(float);
float _approxRunningAverage(float, float);

void _initialiseRandomNumberGenerator(void);
std::vector<int> _createIndexArray(int, bool);