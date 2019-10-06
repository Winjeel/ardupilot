#pragma once

#include "BoardTestCommon.h"
#include "BoardTestUtility.h"

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

// cervello probe test cases - forward declare
static bool _runAllTests_Cervello_Probe(void);
static bool _testMS5611_probe(void);
static bool _testICM20602_probe(void);
static bool _testICM20948_imu_probe(void);
static bool _testICM20948_mag_probe(void);
static bool _testIST8308_probe(void);

// interactive test cases - forward declare
static bool _runAllTests_Cervello_Interactive(void);
static bool _interactiveTest_Accel(void);
static bool _interactiveTest_Gyro(void);
static bool _interactiveTest_Accel_SingleAxis(const float*);
static bool _interactiveTest_Gyro_SingleAxis(const float*);
static bool _interactiveTest_Compass(void);
static bool _interactiveTest_Compass_SingleHeading(const int);
static bool _interactiveTest_Barometer(void);
static bool _interactiveTest_SDCard(void);

// test items
const Test kTestItem[] = {
    { '?', nullptr,              _printInstructions, "Print these instructions.", },
    { '!', nullptr,              _reboot,            "Reboot.", },
    { 'a', nullptr,              _runAll,            "Run all tests.", },

    // All cervello tests
    { '1', "Cervello probe tests",          _runAllTests_Cervello_Probe,            "Cervello - Test if all sensors can be discovered.", },
    { '2', "Cervello interactive tests",    _runAllTests_Cervello_Interactive,      "Cervello - Verify data from all sensors.", },

    // Interactive tests
    { '3', "Cervello accelerometer tests",  _interactiveTest_Accel,                 "Cervello - Test accelerometers.", },
    { '4', "Cervello gyro tests",           _interactiveTest_Gyro,                  "Cervello - Test gyros.", },
    { '5', "Cervello compass tests",        _interactiveTest_Compass,               "Cervello - Test compass.", },
    { '6', "Cervello barometer tests",      _interactiveTest_Barometer,             "Cervello - Test barometer.", },
    { '7', "Cervello SD Card tests",        _interactiveTest_SDCard,                "Cervello - Test SD Card.", },

};
const size_t kNumTestItems = sizeof(kTestItem) / sizeof(kTestItem[0]);