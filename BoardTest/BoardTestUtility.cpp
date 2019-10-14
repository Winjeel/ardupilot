#include "BoardTestUtility.h"

bool _checkGravityAcceleration(float acceleration){
    // This function checks for the reactive force against gravity
    float accelDelta = acceleration + GRAVITY_MSS;

    return abs(accelDelta) < ins_accel_tol;
}

bool _checkRotation(float rotation){
    // Function to check for positive rotation from the current gyro axis
    return rotation > ins_gyro_tol;
}

bool _checkCompassAlignment(float headingDeg){
    // This function checks for if the compass is aligned with magnetic north
    return abs(headingDeg) < compass_northPointing_tol;
}

float _approxRunningAverage(float average, float newSample){
    average -= average / runningAverageSamples;
    average += newSample / runningAverageSamples;
    return average;
    // https://stackoverflow.com/questions/12636613/how-to-calculate-moving-average-without-keeping-the-count-and-data-total
}

void _initialiseRandomNumberGenerator(void){
    std::srand(rngSeed);
}