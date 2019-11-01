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

// Enumerates
enum SerialDeviceList {
    SERIAL0=0, // J12 - USB - OTG
    SERIAL1=1, // J5 - TELEM1 - UART 2 - SERIAL1
    SERIAL2=2, // J6 - TELEM2 - UART 3 - SERIAL2
    SERIAL4=4, // J4 - TELEM 3 - UART 7 - SERIAL4
    SERIAL5=5, // J14-5, J14-7 - TELEM4 - UART 8 - SERIAL5
    SERIAL6=6 // J14-6 - MOTOR_POD  - USART1 Receive (RX) Only - SERIAL6
};

enum PWMDeviceList {
    SERVO1=0, // J2 - SERVO_STBD
    SERVO2=1, // J3 - SERVO_PORT
    SERVO3=2, // J14-8 - ESC_TX
    SERVO4=3, // J1-3 - SERVO_PWM_A
    SERVO5=4, // J1-6 - SERVO_PWM_B
    SERVO6=5, // J1-9 - SERVO_PWM_C
    SERVO7=6, // J1-12 - SERVO_PWM_D
    SERVO8=7, // J1-15- SERVO_PWM_E
    SERVO9=8 // J1-18 - SERVO_PWM_F
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
static bool _PPDSCarrier_serialCommunicationTest_Serial1_Serial2(void);
static bool _PPDSCarrier_serialCommunicationTest_Serial1_Serial4(void);
static bool _PPDSCarrier_serialCommunicationTest_Serial5_Loopback(void);
static bool _PPDSCarrier_serialCommunicationTest_singleCommunication(SerialDeviceList serialDevice1ID, SerialDeviceList serialDevice2ID, AP_HAL::UARTDriver::flow_control hardwareFlowControl);

static bool _PPDSCarrier_serialCommunicationTest_Servo3_Serial6(void);
static bool _PPDSCarrier_serialCommunicationTest_ServoAll_Serial4(void);
static bool _PPDSCarrier_pwmToSerialCommunicationTest_singleCommunication(PWMDeviceList pwmDevice, SerialDeviceList serialDeviceID, bool printFailMsg);

static bool _PPDSCarrier_rcInputTest(void);
static bool _PPDSCarrier_buzzerTest(void);
static bool _PPDSCarrier_safetySwitchTest(void);
static bool _PPDSCarrier_GPSTest(void);

// test items
const Test kTestItem[] = {
    { '?', nullptr,              _printInstructions, "Print these instructions.", },
    { '!', nullptr,              _reboot,            "Reboot.", },

    // All Cervello tests
    { '1', "Cervello probe tests",          _cervello_runAllProbeTests,          "Cervello - Test if all sensors can be discovered.", },
    { '2', "Cervello all tests",            _cervello_runAllTests,               "Cervello - Run all tests.", },

    // Cervello individual tests
    { '3', "Cervello accelerometer tests",  _cervello_AccelTest,                 "Cervello - Run accelerometer test.", },
    { '4', "Cervello gyro tests",           _cervello_GyroTest,                  "Cervello - Run gyro test.", },
    { '5', "Cervello compass tests",        _cervello_CompassTest,               "Cervello - Run compass test.", },
    { '6', "Cervello barometer tests",      _cervello_BarometerTest,             "Cervello - Run barometer test.", },
    { '7', "Cervello SD Card tests",        _cervello_SDCardTest,                "Cervello - Run SD Card test.", },
    { '8', "Cervello RAMTRON tests",        _cervello_RAMTRONTest,               "Cervello - Run RAMTRON test.", },

    // PPDS Carrier individual tests
    { 'q', "PPDS Carrier Serial1 to Serial2 Communication test",            _PPDSCarrier_serialCommunicationTest_Serial1_Serial2,            "PPDS Carrier - Run Serial1 to Serial2 Communication test.", },
    { 'w', "PPDS Carrier Serial1 to Serial4 Communication test",            _PPDSCarrier_serialCommunicationTest_Serial1_Serial4,            "PPDS Carrier - Run Serial1 to Serial4 Communication test.", },
    { 'e', "PPDS Carrier Serial5 Loopback Communication test",              _PPDSCarrier_serialCommunicationTest_Serial5_Loopback,           "PPDS Carrier - Run Serial5 Loopback Communication test.", },

    { 'r', "PPDS Carrier Servo3 to Serial6 Communication test",             _PPDSCarrier_serialCommunicationTest_Servo3_Serial6,             "PPDS Carrier - Run Servo3 to Serial6 Communication test.", },
    { 't', "PPDS Carrier Servos to Serial4 Communication test",             _PPDSCarrier_serialCommunicationTest_ServoAll_Serial4,           "PPDS Carrier - Run All Remaining Servos to Serial4 Communication test.", },

    { 'y', "PPDS Carrier RC Input test",                                    _PPDSCarrier_rcInputTest,                                        "PPDS Carrier - Run RC Input test.", },

    { 'u', "PPDS Carrier Buzzer test",                                      _PPDSCarrier_buzzerTest,                                         "PPDS Carrier - Run Buzzer test.", },
    { 'i', "PPDS Carrier Safety Switch test",                               _PPDSCarrier_safetySwitchTest,                                   "PPDS Carrier - Run Safety Switch test.", },
    { 'o', "PPDS Carrier GPS test",                                         _PPDSCarrier_GPSTest,                                            "PPDS Carrier - Run GPS test.", },
};
const size_t kNumTestItems = sizeof(kTestItem) / sizeof(kTestItem[0]);