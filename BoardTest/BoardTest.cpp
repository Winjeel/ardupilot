/*
  BoardTest
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Baro/AP_Baro.h>

#include <ctype.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();


typedef bool (*TestFn)(void);

typedef struct {
    char key;
    char const * const name; // use nullptr to ignore this test in _runAll()
    TestFn function;
    char const * const description;
} Test;


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

static bool _executeTest(Test const * const test) {
    bool result = false;

    char const * const kPass  = "PASS";
    char const * const kFail  = "FAIL";
    char const * const kError = "ERR ";

    char const * resultStr = kError;

    if (test->function) {
        result = test->function();
        resultStr = result ? kPass : kFail;
    }

    hal.console->printf("[%s] %s\n", resultStr, test->name ? test->name : "???");

    return result;
}


static bool _testBaro(void) {
    const AP_Baro &baro = AP::baro();
    return baro.healthy();
}


static bool _testNotImplemented(void) {
    hal.console->printf("    Not Implemented!\n");
    return false;
}


static bool _printInstructions(void);
static bool _runAll(void);
const Test kTestItem[] = {
    { '?', nullptr,        _printInstructions,     "Print these instructions." },
    { 'a', nullptr,        _runAll,                "Run all tests." },
    { '1', "Barometer",    _testBaro,              "Test the Barometer."},
    { '2', "GPS",          _testNotImplemented,    "Test the GPS."},
    { '3', "IMU",          _testNotImplemented,    "Test the IMU."},
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

        result |= _executeTest(test);
    }

    return result;
}


void setup(void)
{
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


void loop(void)
{
    while (hal.console->available()) {
        char key = hal.console->read();
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
