#pragma once

// Board Includes
#include "AP_BoardConfig/AP_BoardConfig.h"
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Common/AP_FWVersion.h>
#include "AP_HAL/AP_HAL.h"
#include "hwdef.h"
#include <GCS_MAVLink/GCS_Dummy.h>

// Sensor includes
#include <AP_Baro/AP_Baro_MS5611.h> // Baro 1, SPI
#include "AP_InertialSensor/AP_InertialSensor_Invensense.h" // IMU 1, SPI
#include "AP_InertialSensor/AP_InertialSensor_Invensensev2.h" // IMU 2, SPI
#include "AP_Compass/AP_Compass_AK09916.h" // Compass 1, SPI
#include "AP_Compass/AP_Compass_IST8308.h" // Compass 2, I2C

// Utility includes
#include <ctype.h>
#include <AP_Math/definitions.h> // GRAVITY_MSS

// Test constants include
#include "BoardTestConstants.h"