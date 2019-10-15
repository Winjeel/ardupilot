#pragma once

// board test constants

// random number generator seed
#define rngSeed 1234

// test timeouts
#define sensorInitialisationTimeout 5000000 // 5 seconds
#define interactiveTestTimeout 10000000 // 10 seconds
#define probeTestTimeout 5000000 // 5 seconds
#define interactiveTestLoopDelay 5 // 5 microseconds
#define sdCardActivityDelay 500 // 500 microseconds

// moving average samples
#define runningAverageSamples 50

// cervello accelerometer & gyro test constants
#define ins_accel_tol 0.3
#define ins_gyro_tol 0.5

// cervello barometer test constants
#define baro_temp_expectedMin (float)10 // Degrees Celcius
#define baro_temp_expectedMax (float)40 // Degrees Celcius
#define baro_pressure_expectedMin (float)98600 // Pascals -- http://csweather.net/wx14a.php
#define baro_pressure_expectedMax (float)104000 // Pascals

// cervello compass constants
#define compassRotationTolerance (float)90 // Degrees

// cervello RAMTRON size
#define cervelloRamtronSize 16384

// UART constants
#define UARTbaud 5600
#define UARTwriteDelay 20