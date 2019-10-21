#pragma once

#include "BoardTestCommon.h"
#include "BoardTestUtility.h"

// RGB structs
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

// Test structs
typedef bool (*TestFn)(void);

typedef struct {
    char key;
    char const * const name; // use nullptr to ignore this test in _runAll()
    TestFn function;
    char const * const description;
} Test;

char const * kResultStr[]  = {
    "FAIL\n",
    "PASS\n",
};

// main functions - forward declare
void setup();
void loop();

// LED functions - forward declare
static void _initialiseLED(void);
static void _setLED_RGB(struct RGB rgb);
void _updateLED(void);

// test functions - forward declare
static bool _reboot(void);
static bool _executeTest(Test const * const);
static bool _testNotImplemented(void);
static char const * _getResultStr(bool result);
static void _consoleKeypress(void);

static void _printHeader(void);
static bool _printInstructions(void);
static void _printDriverWarning(void);

// cervello probe test cases - forward declare
static bool _cervello_runAllProbeTests(void);
static bool _cervello_probeMS5611(void);
static bool _cervello_probeICM20602(void);
static bool _cervello_probeICM20948imu(void);
static bool _cervello_probeICM20948mag(void);
static bool _cervello_probeIST8308(void);
static bool _cervello_probeRAMTRON(void);

// cervello  test cases - forward declare
static bool _cervello_runAllTests(void);
static bool _cervello_AccelTest(void);
static bool _cervello_GyroTest(void);
static bool _cervello_AccelTest_SingleAxis(float const * const accelSensor);
static bool _cervello_GyroTest_SingleAxis(float const * const gyroSensor);
static bool _cervello_CompassTest(void);
static bool _cervello_CompassTest_SingleHeading(const int i);
static bool _cervello_BarometerTest(void);
static bool _cervello_SDCardTest(void);
static bool _cervello_RAMTRONTest(void);
static bool _cervello_RAMTRONTest_writeValue(uint8_t valueToWrite);
static bool _cervello_RAMTRONTest_writeRandom(void);

// PPDS Carrier test cases - forward declare
static bool _PPDSCarrier_runAllTests(void);
static bool _PPDSCarrier_serialCommunicationTest(int serialDevice1, int serialDevice2, bool enabledHardwareControlFlow);
static bool _PPDSCarrier_buzzerTest(void);
static bool _PPDSCarrier_safetySwitchTest(void);

// test items
const Test kTestItem[] = {
    { '?', nullptr,              _printInstructions, "Print these instructions.", },
    { '!', nullptr,              _reboot,            "Reboot.", },

    // All Cervello tests
    { '1', "Cervello probe tests",          _cervello_runAllProbeTests,      "Cervello - Test if all sensors can be discovered.", },
    { '2', "Cervello all  tests",           _cervello_runAllTests,           "Cervello - Run all tests.", },

    // Cervello individual tests
    { '3', "Cervello accelerometer tests",  _cervello_AccelTest,                 "Cervello - Run accelerometer test.", },
    { '4', "Cervello gyro tests",           _cervello_GyroTest,                  "Cervello - Run gyro test.", },
    { '5', "Cervello compass tests",        _cervello_CompassTest,               "Cervello - Run compass test.", },
    { '6', "Cervello barometer tests",      _cervello_BarometerTest,             "Cervello - Run barometer test.", },
    { '7', "Cervello SD Card tests",        _cervello_SDCardTest,                "Cervello - Run SD Card test.", },
    { '8', "Cervello RAMTRON tests",        _cervello_RAMTRONTest,               "Cervello - Run RAMTRON test.", },

    // All PPDS Carrier Tests
    { '9', "PPDS Carrier Tests",            _PPDSCarrier_runAllTests,                   "PPDS Carrier - Run all tests.", },

};
const size_t kNumTestItems = sizeof(kTestItem) / sizeof(kTestItem[0]);