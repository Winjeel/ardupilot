// Board includes
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
#include <AP_NavEKF2/AP_NavEKF2_core.h> //imu_ring_buffer_t, obs_ring_buffer_t

// Defines
#define accelTol 0.3
#define gyroTol 0.5
#define interactiveTestTimeout 10000000 // 10 seconds
#define runningAverageSamples 50

// RGB class
struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct RGB rgb[] = {
    { 0, 0, 0, }, // black
    { 1, 0, 0, }, // red
    { 1, 1, 0, }, // yellow
    { 0, 1, 0, }, // green
    { 0, 1, 1, }, // cyan
    { 0, 0, 1, }, // blue
    { 1, 0, 1, }, // purple
    { 1, 1, 1, }, // white
};

// Test classes
typedef bool (*TestFn)(void);

typedef struct {
    char key;
    char const * const name; // use nullptr to ignore this test in _runAll()
    TestFn function;
    char const * const description;
} Test;

// main functions - forward declare
void setup();
void loop();

// LED functions - forward declare
static void _initialiseLED(void);
static void _setLED_RGB(struct RGB);
void _updateLED(void);

// test functions - forward declare
static bool _reboot(void);
static bool _runAll(void);
static bool _executeTest(Test const * const);
static bool _testNotImplemented(void);
static char const * _getResultStr(bool);
static void _consoleKeypress(void);

static void _printHeader(void);
static bool _printInstructions(void);

// test cases - forward declare
static bool _testMS5611_interrogate(void);
static bool _testICM20602_interrogate(void);
static bool _testICM20948_imu_interrogate(void);
static bool _testICM20948_mag_interrogate(void);
static bool _testIST8308_interrogate(void);

static bool _testBarometer_sensorData(void);
static bool _testCompass_sensorData(void);
static bool _testINS_sensorData_accel(void);
static bool _testINS_sensorData_gyro(void);

static bool _testINS_accel_xAxis(void);
static bool _testINS_gyro_xAxis(void);
/* static bool _testINS_accel_yAxis(void);
static bool _testINS_accel_zAxis(void); */
static bool _call_generic_AccelTest(void);
static bool _generic_AccelTest(const float*);


// test utilities
bool _checkGravityAcceleration(float);
bool _checkRotation(float);
float _approxRunningAverage(float, float);

// test items
const Test kTestItem[] = {
    { '?', nullptr,              _printInstructions, "Print these instructions.", },
    { '!', nullptr,              _reboot,            "Reboot.", },
    { 'a', nullptr,              _runAll,            "Run all tests.", },

    // interrogation tests
    { '1', "MS5611 (Baro)",      _testMS5611_interrogate,        "Test if the MS-5611 Barometer can be interrogated.", },
    { '2', "ICM20602 (IMU)",     _testICM20602_interrogate,      "Test if the ICM20602 IMU can be interrogated.", },
    { '3', "ICM20948 (IMU)",     _testICM20948_imu_interrogate,  "Test if the ICM20948 IMU can be interrogated.", },
    { '4', "ICM20948 (Compass)", _testICM20948_mag_interrogate,  "Test if the ICM20948 Compass can be interrogated.", },
    { '5', "IST8308 (Compass)",  _testIST8308_interrogate,       "Test if the IST8308 Compass can be interrogated.", },

    // sensor tests
    { '6', "Barometer Data",    _testBarometer_sensorData,       "Test the sensor data from the barometer sensors.", },
    { '7', "Compass Data",      _testCompass_sensorData,         "Test the sensor data from the compass sensors.", },
    { '8', "Accelerometer Data",_testINS_sensorData_accel,       "Test the sensor data from the INS accelerometer sensors.", },
    { '9', "Gyro Data",         _testINS_sensorData_gyro,        "Test the sensor data from the INS gyro sensors.", },

    // running tests
    { 'q', "Running Accel X",   _testINS_accel_xAxis,        "Interactive X axis acceleration.", },
    { 'w', "Running Gyro X",   _testINS_gyro_xAxis,        "Interactive X axis gyro.", },
    { 'e', "Running Accel",   _call_generic_AccelTest,        "Interactive INS tests.", },
/*     { 'w', "Running Accel Y",   _testINS_accel_yAxis,        "Interactive Y axis acceleration.", },
    { 'e', "Running Accel Z",   _testINS_accel_zAxis,        "Interactive Z axis acceleration.", }, */
};
const size_t kNumTestItems = sizeof(kTestItem) / sizeof(kTestItem[0]);