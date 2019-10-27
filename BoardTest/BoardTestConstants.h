#pragma once

// board test constants

// random number generator seed
#define rngSeed 1234

// test timeouts
#define sensorInitialisationTimeout 5000000 // 5 seconds
#define testTimeout 10000000 // 10 seconds
#define testLoopDelay 5 // 5 microseconds
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

// Port definitions
#define SERIAL1 1 // J5 - TELEM1 - UART 2 - SERIAL1
#define SERIAL2 2 // J6 - TELEM2 - UART 3 - SERIAL2
#define SERIAL4 4 // J4 - TELEM 3 - UART 7 - SERIAL4
#define SERIAL5 5 // J14-5, J14-7 - TELEM4 - UART 8 - SERIAL5
#define SERIAL6 6 // J14-6 - MOTOR_POD  - USART1 Receive (RX) Only - SERIAL6

#define SERVO1 1 // J2 - SERVO_STBD
#define SERVO2 2 // J3 - SERVO_PORT
#define SERVO3 3 // J14-8 - ESC_TX
#define SERVO4 4 // J1-3 - SERVO_PWM_A
#define SERVO5 5 // J1-6 - SERVO_PWM_B
#define SERVO6 6 // J1-9 - SERVO_PWM_C
#define SERVO7 7 // J1-12 - SERVO_PWM_D
#define SERVO8 8 // J1-15- SERVO_PWM_E
#define SERVO9 9 // J1-18 - SERVO_PWM_F