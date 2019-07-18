/*
  BoardTest
 */

#include <utility> // for std::move
#include <ctype.h>

#include "hwdef.h"

#include <AP_HAL/AP_HAL.h>


#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_SerialManager/AP_SerialManager.h>

// sensor includes
#include "AP_Baro/AP_Baro.h"
#include "AP_Baro/AP_Baro_MS5611.h"
#include "AP_InertialSensor/AP_InertialSensor.h"
#include "AP_InertialSensor/AP_InertialSensor_Invensense.h"
#include "AP_InertialSensor/AP_InertialSensor_Invensensev2.h"
#include "AP_Compass/AP_Compass.h"
#include "AP_Compass/AP_Compass_AK09916.h"
#include "AP_Compass/AP_Compass_IST8308.h"


#if APJ_BOARD_ID != 11
    // TODO: Fix the board ID (currently 11=FMUv4).
    #error This BoardTest is currently only applicable for Cervello boards!
#endif

// linking fails without this all_stream_entries[] definition.
#include <GCS_MAVLink/GCS.h>
const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] {};

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_BoardConfig boardConfig;
static AP_SerialManager serialManager;

// dummy sensor managers for probe functions.
static AP_Baro kDummyBaro;
static AP_InertialSensor kDummyIMU;

typedef bool (*TestFn)(void);

typedef struct {
    char key;
    char const * const name; // use nullptr to ignore this test in _runAll()
    TestFn function;
    char const * const description;
} Test;


static char const * _getResultStr(bool result) {
    char const * kResultStr[]  = {
        "FAIL",
        "PASS",
    };

    return kResultStr[!!result];
}


static void _printHeader(void) {
    char const * const kHeader[] = {
        "------------------------------------------------------------------------------\n",
        "                                                                              \n",
        "           .g8\"\"\"bgd   .g8\"\"8q. `7MM\"\"\"Mq.`7MMF'   `7MF' .g8\"\"8q.             \n",
        "         .dP'     `M .dP'    `YM. MM   `MM. `MA     ,V .dP'    `YM.           \n",
        "         dM'       ` dM'      `MM MM   ,M9   VM:   ,V  dM'      `MM           \n",
        "         MM          MM        MM MMmmdM9     MM.  M'  MM        MM           \n",
        "         MM.         MM.      ,MP MM  YM.     `MM A'   MM.      ,MP           \n",
        "         `Mb.     ,' `Mb.    ,dP' MM   `Mb.    :MM;    `Mb.    ,dP'           \n",
        "           `\"bmmmd'    `\"bmmd\"' .JMML. .JMM.    VF       `\"bmmd\"'             \n",
        "                                                                              \n",
        "                                                BoardTest v0.0.1              \n",
        "                                                                              \n",
        "------------------------------------------------------------------------------\n",
        "\n",
    };
    const size_t kNumLines = sizeof(kHeader) / sizeof(kHeader[0]);
    for (int i = 0; i < kNumLines; i++) {
        hal.console->printf(kHeader[i]);
    }
}


static bool _testNotImplemented(void) {
    hal.console->printf("    [%s] Not Implemented!\n", _getResultStr(false));
    return false;
}


static bool _executeTest(Test const * const test) {
    bool result = true;

    hal.console->printf("Running test %s:\n", test->name ? test->name : "???");

    if (test->function) {
        result = test->function();
    } else {
        result = _testNotImplemented();
    }

    hal.console->printf("[%s] %s\n", _getResultStr(result), test->name ? test->name : "???");

    return result;
}

// Baro 1, SPI
static bool _testMS5611(void) {
    bool result = false;

    AP_Baro_Backend * backend = HAL_BARO_1_PROBE(kDummyBaro);
    HAL_BARO_1_DRIVER * ms5611 = static_cast<AP_Baro_MS56XX *>(backend);

    // Backends are painful. The probe function runs the init, and if
    // successful, returns a pointer.
    result = (ms5611 != nullptr);

    return result;
}

// IMU 1, SPI
static bool _testICM20602(void) {
    bool result = false;

    AP_InertialSensor_Backend * backend = HAL_INS_1_PROBE(kDummyIMU);
    HAL_INS_1_DRIVER * icm20602 = static_cast<HAL_INS_1_DRIVER *>(backend);

    // Backends are painful. The probe function runs the init, and if
    // successful, returns a pointer.
    result = (icm20602 != nullptr);

    return result;
}

// IMU 2, SPI
static bool _testICM20948_imu(void) {
    bool result = false;

    AP_InertialSensor_Backend * backend = HAL_INS_2_PROBE(kDummyIMU);
    HAL_INS_2_DRIVER * icm20948_imu = static_cast<HAL_INS_2_DRIVER *>(backend);

    // Backends are painful. The probe function runs the init, and if
    // successful, returns a pointer.
    result = (icm20948_imu != nullptr);

    return result;
}

// Compass 1, SPI
static bool _testICM20948_mag(void) {
    bool result = false;

    AP_Compass_Backend * backend = HAL_MAG_1_PROBE;
    HAL_MAG_1_DRIVER * icm20948_mag = static_cast<HAL_MAG_1_DRIVER *>(backend);

    // Backends are painful. The probe function runs the init, and if
    // successful, returns a pointer.
    result = (icm20948_mag != nullptr);

    return result;
}

// Compass 2, I2C
static bool _testIST8308(void) {
    bool result = false;

    AP_Compass_Backend * backend = HAL_MAG_2_PROBE;
    HAL_MAG_2_DRIVER * ist8308 = static_cast<HAL_MAG_2_DRIVER *>(backend);

    // Backends are painful. The probe function runs the init, and if
    // successful, returns a pointer.
    result = (ist8308 != nullptr);

    return result;
}

static bool _reboot(void) {
    bool hold_in_bootloader = false;
    hal.scheduler->reboot(hold_in_bootloader);
    return true;
}

static bool _printInstructions(void);
static bool _runAll(void);
const Test kTestItem[] = {
    { '?', nullptr,              _printInstructions, "Print these instructions.", },
    { '!', nullptr,              _reboot,            "Reboot.", },
    { 'a', nullptr,              _runAll,            "Run all tests.", },

    { '1', "MS5611 (Baro)",      _testMS5611,        "Test the MS-5611 Barometer.", },
    { '2', "ICM20602 (IMU)",     _testICM20602,      "Test the ICM20602 IMU.", },
    { '3', "ICM20948 (IMU)",     _testICM20948_imu,  "Test the ICM20948 IMU.", },
    { '4', "ICM20948 (Compass)", _testICM20948_mag,  "Test the ICM20948 Compass.", },
    { '5', "IST8308 (Compass)",  _testIST8308,       "Test the IST8308 Compass.", },
};
const size_t kNumTestItems = sizeof(kTestItem) / sizeof(kTestItem[0]);


static bool _printInstructions(void) {
    hal.console->printf("Press a key to run one of the following commands:\n");
    for (int i = 0; i < kNumTestItems; i++) {
        hal.console->printf("    %c  %s\n", kTestItem[i].key, kTestItem[i].description);
    }

    return true;
}


static bool _runAll(void) {
    bool result = true;

    for (int i = 0; i < kNumTestItems; i++) {
        Test const * const test = &kTestItem[i];
        if (!test->name) {
            continue;
        }

        result =_executeTest(test) && result;
    }

    return result;
}


static void _driverInit(void) {
    // initialise serial port
    serialManager.init_console();

    while (!hal.console->is_initialized()) {
        hal.scheduler->delay(100);
    }

    hal.console->printf("\nInit %s"
                        "\nFree RAM: %lu\n",
                        AP::fwversion().fw_string,
                        (unsigned)hal.util->available_memory());

    // initialise serial ports
    serialManager.init();

    serialManager.set_blocking_writes_all(false);

    // setup any board specific drivers
    boardConfig.init();
}

static void _consoleInit(void) {
    while (!hal.console->is_initialized()) {
        hal.scheduler->delay(100);
    }

    hal.scheduler->delay(2000);

    while (hal.console->available()) {
        hal.console->read(); // flush read buffer
    }

    _printHeader();
    _printInstructions();
    hal.console->printf("\nEnter command: ");
}


static void _led_init(void) {
    // when HAL_GPIO_LED_ON is 0 then we must not use pinMode()
    // as it could remove the OPENDRAIN attribute on the pin
#if HAL_GPIO_LED_ON != 0
    hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_B_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_C_LED_PIN, HAL_GPIO_OUTPUT);
#endif
    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
}

struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

static void _led_set_rgb(struct RGB rgb) {
    hal.gpio->write(HAL_GPIO_A_LED_PIN, (rgb.r > 0) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_B_LED_PIN, (rgb.g > 0) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_C_LED_PIN, (rgb.b > 0) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
}


void setup(void)
{
    _led_init();
    _driverInit();
    _consoleInit();
}

static uint32_t sNow_ms = 0;
void loop(void)
{
    EXPECT_DELAY_MS(1000);

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
    const size_t kNumColours = sizeof(rgb) / sizeof(rgb[0]);

    uint32_t now_ms = AP_HAL::millis();
    const uint32_t kLED_delta_ms = 512;
    if ((now_ms - sNow_ms) > kLED_delta_ms) {
        static uint8_t j = 0;
        _led_set_rgb(rgb[j]);
        j = (j + 1) % kNumColours;

        sNow_ms += kLED_delta_ms;
    }

    while (hal.console->available()) {
        char key = hal.console->read();
        if (key == '\n' || key == '\r') {
            // ignore enter key
            continue;
        }
        hal.console->write(key);

        Test const * test = nullptr;
        for (int i = 0; i < kNumTestItems; i++) {
            if (kTestItem[i].key == key) {
                test = &kTestItem[i];
                break;
            }
        }

        key = isprint(key) ? key : 0;
        hal.console->printf("\n");
        if (test) {
            if (test->name) {
                _executeTest(test);
            } else if (test->function) {
                test->function();
            } else {
                hal.console->printf("Bad mapping for command key '%c'!\n", key);
            }
        } else {
            hal.console->printf("Unknown command key '%c'! Press '?' to see available commands.\n", key);
        }

        hal.console->printf("\nEnter command: ");
    }

    hal.scheduler->delay(10);
}

AP_HAL_MAIN();
