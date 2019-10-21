/*
  Cervello board test
*/
#include "BoardTest.h"

#if APJ_BOARD_ID != 1688
    #error This BoardTest is currently only applicable for Cervello boards!
#endif

// Board classes
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
static AP_BoardConfig boardConfig;
static AP_SerialManager serialManager;
static AP_Notify notify;

// Sensor classes
static AP_Baro barometer;
static Compass compass;
static AP_InertialSensor ins;

// On-board memory classes
AP_RAMTRON ramtron;
AP_Int32 log_bitmask;
AP_Logger logger{log_bitmask};

// Track dirty driver state
bool dirtyDriverState = false;

// initialisation functions
static void _initialiseCervello(void){
    // function to initialise the Cervello
    
    // initialise serial port
    serialManager.init_console();

    while (!hal.console->is_initialized()) {
        hal.scheduler->delay(1000);
    }

    // initialise serial ports
    serialManager.init();

    // initialise Cervello
    boardConfig.init();
    hal.scheduler->delay(1000);

    // initialisation notification system
    notify.init();    
};

static void _initialiseConsole(void) {
    hal.scheduler->delay(5000);

    while (hal.console->available()) {
        hal.console->read(); // flush read buffer
    }

    _printHeader();
    _printInstructions();
    hal.console->printf("\nEnter command: ");
}

static void _initialiseBarometer(void){
    // Function to initialise the barometer

    // Initialise the barometer if it has not been initialised, or is not healthy
    if (barometer.all_healthy()){
        return;
    }

    EXPECT_DELAY_MS(sensorInitialisationTimeout);

    barometer.init();
    barometer.calibrate();
};

static void _initialiseCompass(void){
    // Function to initialise the compass

    // Initialise the compass if it has not been initialised, or is not healthy
    if (compass.healthy()){
        return;
    }

    EXPECT_DELAY_MS(sensorInitialisationTimeout);
    
    compass.init();
    for (uint8_t i = 0; i < compass.get_count(); i++) {
        compass.set_and_save_offsets(i, Vector3f(0, 0, 0));
    }
    compass.set_declination(ToRad(0.0f));
};

static void _initialiseINS(void){
    // Function to initialise the INS

    // Initialise the INS if it has not been initialised, or is not healthy
    if (ins.get_accel_health_all() && ins.get_gyro_health_all()){
        return;
    }

    // Initialise INS if not initialised/not healthy
    EXPECT_DELAY_MS(sensorInitialisationTimeout);
    ins.init(100);

}

static bool _initialiseLogger(void){
    // Function to initialise the logging system

    // Initialise the logging system if it has not been initialised
    if (!logger.WritesEnabled()){
        log_bitmask = (uint32_t)-1;
        logger.Init(log_structure, 0);

        logger.EraseAll();
        // Removes all existing log files, however throws debug
        // message in console: Unable to fetch Log File Size: ENOENT

        hal.scheduler->delay(sdCardActivityDelay);
    }

    return logger.WritesEnabled();

}

static bool _initialiseRAMTRON(void){
    bool ramtronStatus = ramtron.init();
    if (ramtronStatus){
        hal.console->printf("RAMTRON Memory: %uKB\n", (uint)ramtron.get_size());
    }
    return ramtronStatus;
}

static void _printHeader(void) {
    char const * const kHeader[] = {
        "\n",
        "------------------------------------------------------------------------------\n",
        "\n",
        "           .g8\"\"\"bgd   .g8\"\"8q. `7MM\"\"\"Mq.`7MMF'   `7MF' .g8\"\"8q.             \n",
        "         .dP'     `M .dP'    `YM. MM   `MM. `MA     ,V .dP'    `YM.           \n",
        "         dM'       ` dM'      `MM MM   ,M9   VM:   ,V  dM'      `MM           \n",
        "         MM          MM        MM MMmmdM9     MM.  M'  MM        MM           \n",
        "         MM.         MM.      ,MP MM  YM.     `MM A'   MM.      ,MP           \n",
        "         `Mb.     ,' `Mb.    ,dP' MM   `Mb.    :MM;    `Mb.    ,dP'           \n",
        "           `\"bmmmd'    `\"bmmd\"' .JMML. .JMM.    VF       `\"bmmd\"'             \n",
        "\n\n",
    };
    const size_t kNumLines = sizeof(kHeader) / sizeof(kHeader[0]);
    for (int i = 0; i < kNumLines; i++) {
        hal.console->printf(kHeader[i]);
    }

    hal.console->printf("%57s\n", AP::fwversion().fw_string);
    hal.console->printf("\n------------------------------------------------------------------------------\n\n");
}

static bool _printInstructions(void) {
    hal.console->printf("Press a key to run one of the following commands:\n");
    for (int i = 0; i < kNumTestItems; i++) {
        hal.console->printf("    %c  %s\n", kTestItem[i].key, kTestItem[i].description);
    }

    return true;
}

static void _printDriverWarning(void){
        char const * const kWarning[] = {
        "\n",
        "------------------------------------------------------------------------------\n",
        "----------------------------------Warning-------------------------------------\n",
        "-----------------Cervello sensor drivers are in a dirty state-----------------\n",
        "-----------------Reset Cervello before running further tests------------------\n",
        "------------------------------------------------------------------------------\n",
        };
    const size_t kNumLines = sizeof(kWarning) / sizeof(kWarning[0]);

    for (int i = 0; i < kNumLines; i++) {
        hal.console->printf(kWarning[i]);
    }
 }

void setup()
{
    // initialise Cervello
    _initialiseCervello();
    _initialiseLED();
    
    // initialise test console
    _initialiseConsole();

    // set up timer to count time in microseconds
    AP_HAL::micros();
}

void loop()
{
    // terminate program if console fails to initialize
    if (!hal.console->is_initialized()) {
        return;
    }

    // update LED color
    _updateLED();

    // wait for keypress from user
    _consoleKeypress();

    hal.scheduler->delay(10);

}

// LED functions
static void _initialiseLED(void) {
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

static void _setLED_RGB(struct RGB rgb) {
    hal.gpio->write(HAL_GPIO_A_LED_PIN, (rgb.r > 0) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_B_LED_PIN, (rgb.g > 0) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_C_LED_PIN, (rgb.b > 0) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
}

void _updateLED(void){
    static uint32_t sNow_ms = 0;
    const size_t kNumColours = sizeof(rgb) / sizeof(rgb[0]);

    uint32_t now_ms = AP_HAL::millis();
    const uint32_t kLED_delta_ms = 512;
    if ((now_ms - sNow_ms) > kLED_delta_ms) {
        static uint8_t j = 0;
        _setLED_RGB(rgb[j]);
        j = (j + 1) % kNumColours;
        sNow_ms += kLED_delta_ms;
    }
}

// test functions
static bool _reboot(void) {
    const bool hold_in_bootloader = false;
    hal.scheduler->reboot(hold_in_bootloader);
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

static bool _executeTest(Test const * const test) {

    // Verify that the Cervello sensor drivers do not exist in a dirty state
    if (dirtyDriverState){
        _printDriverWarning();
        return false;
    }

    // terminate program if console fails to initialize
    if (!hal.console->is_initialized()) {
        return false;
    }

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

static bool _testNotImplemented(void) {
    hal.console->printf("    [%s] Not Implemented!\n", _getResultStr(false));
    return false;
}

static char const * _getResultStr(bool result) {
    char const * kResultStr[]  = {
        "FAIL",
        "PASS",
    };
    return kResultStr[!!result];
}

static void _consoleKeypress(void){

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
}

static bool _cervello_runAllProbeTests(void){
    // Function to run all probe tests on the Cervello
    bool summaryTestResult = true;
    bool testResult;

    EXPECT_DELAY_MS(probeTestTimeout);

    hal.console->printf("Probing MS5611 (Baro - SPI)\n");
    testResult = _cervello_probeMS5611();
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult;

    hal.console->printf("Probing ICM20602 (IMU1 - SPI)\n");
    testResult = _cervello_probeICM20602();
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult;

    hal.console->printf("Probing ICM20948 (IMU2 - SPI)\n");
    testResult = _cervello_probeICM20948imu();
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult;

    hal.console->printf("Probing ICM20948 (Compass - SPI)\n");
    testResult = _cervello_probeICM20948mag();
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult;

    hal.console->printf("Probing IST8308 (Compass - I2C)\n");
    testResult = _cervello_probeIST8308();
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult;

    hal.console->printf("Probing RAMTRON\n");
    testResult = _cervello_probeRAMTRON();
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult;

    dirtyDriverState = true;
    return summaryTestResult;
}

static bool _cervello_runAllInteractiveTests(void){
    // function to run all interactive tests on the Cervello
    bool summaryTestResult = true;
    
    // run the Accelerometer and Gyro tests
    summaryTestResult &= _cervello_interactiveAccel();
    summaryTestResult &= _cervello_interactiveGyro();

    // run the compass tests
    summaryTestResult &= _cervello_interactiveCompass();

    // run the barometer tests
    summaryTestResult &= _cervello_interactiveBarometer();

    // run the SD Card tests
    summaryTestResult &= _cervello_interactiveSDCard();

    // run the RAMTRON tests
    summaryTestResult &= _cervello_interactiveRAMTRON();

    return summaryTestResult;
}

static bool _PPDSCarrier_runAllTests(void){
    // function to run all tests on the PPDS Carrier
    bool summaryTestResult = true;
    bool testResult;

    // SERIAL0	USB	OTG
    // SERIAL1	SERIAL1 (J5)	USART2
    // SERIAL2	SERIAL2 (J6)	USART3
    // SERIAL3	GPS (J13)	UART4
    // SERIAL4	TELEM3 (J4)	UART7
    // SERIAL5	TELEM4 (J14-5, J14-7)	UART8
    // SERIAL6	MOTOR_POD (J14-6)	USART1 Receive (RX) Only

    // UART communication tests
    const int SERIAL1 = 1; // J5 - TELEM1 - UART 2 - SERIAL1
    const int SERIAL2 = 2; // J6 - TELEM2 - UART 3 - SERIAL2
    const int SERIAL4 = 4; // J4 - TELEM 3 - UART 7 - SERIAL4
    const int SERIAL5 = 5; // J14-5, J14-7 - TELEM4 - UART 8 - SERIAL5

     // Test Serial 1 <-> Serial 2 communication
    hal.console->printf("Testing PPDS Carrier crosstalk from serial device %i to %i --- ", SERIAL1, SERIAL2);
    testResult = _PPDSCarrier_serialCommunicationTest(SERIAL1, SERIAL2, false);
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult;
    hal.console->printf("Testing PPDS Carrier crosstalk from serial device %i to %i --- ", SERIAL2, SERIAL1);
    testResult = _PPDSCarrier_serialCommunicationTest(SERIAL2, SERIAL1, false);
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult; hal.console->printf("\n");

    // Test Serial 1 <-> Serial 2 communication with hardware flow control
    hal.console->printf("Testing PPDS Carrier hardware flow control from serial device %i to %i --- ", SERIAL1, SERIAL2);
    testResult = _PPDSCarrier_serialCommunicationTest(SERIAL1, SERIAL2, true);
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult;
    hal.console->printf("Testing PPDS Carrier hardware flow control from serial device %i to %i --- ", SERIAL2, SERIAL1);
    testResult = _PPDSCarrier_serialCommunicationTest(SERIAL2, SERIAL1, true);
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult; hal.console->printf("\n");

    // Test Serial 1 <-> Serial 4 communication
    hal.console->printf("Testing PPDS Carrier crosstalk from serial device %i to %i --- ", SERIAL1, SERIAL4);
    testResult = _PPDSCarrier_serialCommunicationTest(SERIAL1, SERIAL4, false);
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult;
    hal.console->printf("Testing PPDS Carrier crosstalk from serial device %i to %i --- ", SERIAL4, SERIAL1);
    testResult = _PPDSCarrier_serialCommunicationTest(SERIAL4, SERIAL1, false);
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult; hal.console->printf("\n");

    // Test Serial 5 loopback
    hal.console->printf("Testing PPDS Carrier crosstalk from serial device %i to %i --- ", SERIAL5, SERIAL5);
    testResult = _PPDSCarrier_serialCommunicationTest(SERIAL5, SERIAL5, false);
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult; hal.console->printf("\n");

    // Test Buzzer
    hal.console->printf("Testing PPDS Carrier Buzzer --- ");
    testResult = _PPDSCarrier_buzzerTest();
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult;

    // Test safety switch
    hal.console->printf("Testing PPDS Carrier Safety Switch --- ");
    testResult = _PPDSCarrier_safetySwitchTest();
    hal.console->printf(kResultStr[testResult]);
    summaryTestResult &= testResult;

    return summaryTestResult;
}

// test cases - cervello probe
static bool _cervello_probeMS5611(void) { // Baro 1, SPI
    bool result = false;
    static AP_Baro_Backend *sMS5611_backend = nullptr;

    if (!sMS5611_backend) {
        // Backends are painful. The probe function runs the init, and if
        // successful, returns a pointer.
        sMS5611_backend = HAL_BARO_1_PROBE(barometer);
    }

    if (sMS5611_backend) {
        HAL_BARO_1_DRIVER &ms5611 = static_cast<HAL_BARO_1_DRIVER&>(*sMS5611_backend); // MS5611 doesn't have a from() function
        (void)ms5611;
        result = true;
    }
    return result;
}

static bool _cervello_probeICM20602(void) { // IMU 1, SPI
    bool result = false;
    static AP_InertialSensor_Backend *sICM20602_backend = nullptr;

    if (!sICM20602_backend) {
        // Backends are painful. The probe function runs the init, and if
        // successful, returns a pointer.
        sICM20602_backend = HAL_INS_1_PROBE(ins);
    }

    if (sICM20602_backend) {
        HAL_INS_1_DRIVER &icm20602 = HAL_INS_1_DRIVER::from(*sICM20602_backend);
        icm20602.start();
        result = icm20602.update();
    }
    return result;
}

static bool _cervello_probeICM20948imu(void) { // IMU 2, SPI
    bool result = false;
    static AP_InertialSensor_Backend *sICM20948_imu_backend = nullptr;

    if (!sICM20948_imu_backend) {
        // Backends are painful. The probe function runs the init, and if
        // successful, returns a pointer.
        sICM20948_imu_backend = HAL_INS_2_PROBE(ins);
    }

    if (sICM20948_imu_backend) {
        HAL_INS_2_DRIVER &icm20948 = HAL_INS_2_DRIVER::from(*sICM20948_imu_backend);
        icm20948.start();
        result = icm20948.update();
    }
    return result;
}

static bool _cervello_probeICM20948mag(void) { // Compass 1, SPI
    bool result = false;
    static AP_Compass_Backend *sICM20948_mag_backend = nullptr;

    if (!sICM20948_mag_backend) {
        // Backends are painful. The probe function runs the init, and if
        // successful, returns a pointer.
        sICM20948_mag_backend = HAL_MAG_1_PROBE;
    }

    if (sICM20948_mag_backend) {
        HAL_MAG_1_DRIVER &icm20948_mag = static_cast<HAL_MAG_1_DRIVER&>(*sICM20948_mag_backend);
        (void)icm20948_mag;
        result = true;
    }
    return result;
}

static bool _cervello_probeIST8308(void) { // Compass 2, I2C
    bool result = false;
    static AP_Compass_Backend * sIST8308_backend = nullptr;

    if (!sIST8308_backend) {
        // Backends are painful. The probe function runs the init, and if
        // successful, returns a pointer.
        sIST8308_backend = HAL_MAG_2_PROBE;
    }

    if (sIST8308_backend) {
        HAL_MAG_2_DRIVER &ist8308 = static_cast<HAL_MAG_2_DRIVER&>(*sIST8308_backend);
        (void)ist8308;
        result = true;
    }
    return result;
}

static bool _cervello_probeRAMTRON(void){ // RAMTRON
    return ramtron.init();
}

// test cases - cervello interactive
static bool _cervello_interactiveAccel(void){
    // Initialise the INS
    _initialiseINS();

    // Verify that the INS sensors exist before continuing
    if (ins.get_accel_count() < 1){
        hal.console->printf("No INS sensors found\n");
        return false;
    }
    hal.console->printf("\n");

    // Setup variable to track test result
    bool summaryTestResult = true;

    // Loop through each axis - X/Y/Z
    for (int i = 0; i < 3; i++){

        char axis[] = {'X', 'Y', 'Z', };
        hal.console->printf("Orient the board with the %c axis facing down\n", axis[i]);

        // Loop through each accelerometer
        for (uint8_t j = 0; j < ins.get_accel_count(); j++) {

            hal.console->printf("Testing accelerometer %i --- ",j);
            bool testResult =_cervello_interactiveAccel_SingleAxis(&ins.get_accel(j)[i]);
            hal.console->printf(kResultStr[testResult]);
            summaryTestResult &= testResult;

        } // Accelerometer loop
        hal.console->printf("\n");
    } // Axis loop

    return summaryTestResult;
}

static bool _cervello_interactiveGyro(void){
    // Initialise the INS
    _initialiseINS();

    // Verify that the INS sensors exist before continuing
    if (ins.get_gyro_count() < 1){
        hal.console->printf("No INS sensors found\n");
        return false;
    }

    // Setup variable to track test result
    bool summaryTestResult = true;

    // Loop through each axis - X/Y/Z
    for (int i = 0; i < 3; i++){

        char axis[] = {'X', 'Y', 'Z', };
        hal.console->printf("Rotate the board clockwise around the positive %c axis\n", axis[i]);

        // Loop through each gyro
        for (uint8_t j = 0; j < ins.get_gyro_count(); j++) {

            hal.console->printf("Testing gyro %i --- ",j);
            bool testResult =_cervello_interactiveGyro_SingleAxis(&ins.get_gyro(j)[i]);
            hal.console->printf(kResultStr[testResult]);
            summaryTestResult &= testResult;

        } // Gyro loop
        hal.console->printf("\n");
    } // Axis loop

    return summaryTestResult;
}

static bool _cervello_interactiveAccel_SingleAxis(float const * const accelSensor){
    // Expect delay based on timeout duration;
    EXPECT_DELAY_MS((int)interactiveTestTimeout);

    // Setup test duration
    uint32_t testStartTime = AP_HAL::micros();
    uint32_t testEndTime = testStartTime + (uint32_t)interactiveTestTimeout;

    // Setup variable to track running average
    float runningAverage = 0;

    // Poll the gyro data
    while(AP_HAL::micros() < testEndTime){

        // Update accelerometer and retrieve data
        ins.update();

        // Update the running average
        float accelData = *accelSensor;
        runningAverage = _approxRunningAverage(runningAverage, accelData);

        // Check if accelerometer is aligned with gravity
        if (_checkGravityAcceleration(runningAverage)){
            // If accelerometer is aligned, pass test and continue
            return true;
        }
        hal.scheduler->delay(interactiveTestLoopDelay);
    }
    return false;
}

static bool _cervello_interactiveGyro_SingleAxis(float const * const gyroSensor){
    // Expect delay based on timeout duration;
    EXPECT_DELAY_MS((int)interactiveTestTimeout);

    // Setup test duration
    uint32_t testStartTime = AP_HAL::micros();
    uint32_t testEndTime = testStartTime + (uint32_t)interactiveTestTimeout;

    // Setup variable to track running average
    float runningAverage = 0;

    // Poll the gyro data
    while(AP_HAL::micros() < testEndTime){

        // Update accelerometer and retrieve data
        ins.update();

        // Update the running average
        float gyroData = *gyroSensor;
        runningAverage = _approxRunningAverage(runningAverage, gyroData);

        // hal.console->printf("Running average %.2f\n",runningAverage);

        // Check if gyro detects positive rotation
        if (_checkRotation(runningAverage)){
            // If accelerometer is aligned, pass test and continue
            return true;
        }
        hal.scheduler->delay(interactiveTestLoopDelay);
    }
    return false;
}

static bool _cervello_interactiveBarometer(void){
    // Initialise the barometer
    _initialiseBarometer();

    // Verify that the barometers exist before continuing
    if (barometer.num_instances() < 1){
        hal.console->printf("No barometer sensors found\n");
        return false;
    }

    // verify that the barometer is healthy before continuing
    if (!barometer.all_healthy()){
        hal.console->print("Barometer not healthy\n");
        return false;
    }

    // setup variable to track test results
    bool summaryTestResult = true;

    // update the barometer for new sensor information
    barometer.accumulate();
    barometer.update();

    // verifying temperature data is within reasonable expected values
    hal.console->printf("Testing barometer temperature within range %.1fC to %.1fC -- Baro temperature: %.1fC -- ", baro_temp_expectedMin, baro_temp_expectedMax,barometer.get_temperature());
    bool testResult_checkTempMax = barometer.get_temperature() <= baro_temp_expectedMax;
    summaryTestResult &= testResult_checkTempMax;
    bool testResult_checkTempMin = barometer.get_temperature() >= baro_temp_expectedMin;
    summaryTestResult &= testResult_checkTempMin;

    if (testResult_checkTempMax && testResult_checkTempMin){
        hal.console->printf("PASS\n");
    }
    else if (!testResult_checkTempMax){
        // If measured temperature is too high
        hal.console->printf("FAIL\nBarometer temperature too high - %.1fC\n", barometer.get_temperature());
    }
    else {
        // If measured temperature is too low
        hal.console->printf("FAIL\nBarometer temperature too low - %.1fC\n", barometer.get_temperature());
    }

    // verifying pressure data is within reasonable expected values
    hal.console->printf("Testing barometer pressure within range %.1fPa to %.1fPa -- Baro pressure: %.1fPa -- ", baro_pressure_expectedMin, baro_pressure_expectedMax, barometer.get_pressure());
    bool testResult_checkPressureMax = barometer.get_pressure() <= baro_pressure_expectedMax;
    summaryTestResult &= testResult_checkPressureMax;
    bool testResult_checkPressureMin = barometer.get_pressure() >= baro_pressure_expectedMin;
    summaryTestResult &= testResult_checkPressureMin;

    if (testResult_checkPressureMax && testResult_checkPressureMin){
        hal.console->printf("PASS\n\n");
    }
    else if (!testResult_checkPressureMax){
        // If measured pressure is too high
        hal.console->printf("FAIL\nBarometer pressure too high - %.1fPa\n\n", barometer.get_pressure());
    }
    else {
        // If measured pressure is too low
        hal.console->printf("FAIL\nBarometer pressure too low - %.1fPa\n\n", barometer.get_pressure());
    }

    return summaryTestResult;
}

static bool _cervello_interactiveCompass(void){
    // Initialise the compass
    _initialiseCompass();

    // Verify that the compass sensors exist before continuing
    if (compass.get_count() < 1){
        hal.console->printf("No compass sensors found\n");
        return false;
    }

    // Check all compass sensors are healthy
    for (uint8_t i = 0; i < compass.get_count(); i++) {
        if (!compass.healthy(i)){
            hal.console->printf("Compass %i not healthy\n",i);
            return false;
        }
    }

    // Setup variable to track test result
    bool summaryTestResult = true;

    // Loop through each compass
    hal.console->printf("Rotate the board %.0f Degrees around the Z axis\n", compassRotationTolerance);
    for (uint8_t j = 0; j < compass.get_count(); j++) {

        hal.console->printf("Testing compass %i --- ",j);
        bool testResult =_cervello_interactiveCompass_SingleHeading(j);
        (testResult) ? hal.console->printf("PASS\n") : hal.console->printf("FAIL\n");
        summaryTestResult &= testResult;

    } // Compass loop
    hal.console->printf("\n");

    return summaryTestResult;
}

static bool _cervello_interactiveCompass_SingleHeading(const int i){
    // Expect delay based on timeout duration;
    EXPECT_DELAY_MS((int)interactiveTestTimeout);

    // Setup test duration
    uint32_t testStartTime = AP_HAL::micros();
    uint32_t testEndTime = testStartTime + (uint32_t)interactiveTestTimeout;

    // Calculate a reference compass heading
    compass.read();
    Matrix3f dcm_matrix;
    dcm_matrix.from_euler(0, 0, 0); // roll pitch yaw 0
    float referenceHeading = ToDeg(compass.calculate_heading(dcm_matrix, i));
    // NOTE: this may not be the magnetic heading as that the compass may not have been calibrated

    // Poll the compass data
    while(AP_HAL::micros() < testEndTime){

        // Update compass and retrieve data
        compass.read();

        // calculate the new heading
        float newHeading = ToDeg(compass.calculate_heading(dcm_matrix, i));

        // Check if compass has been rotated
        if (abs(referenceHeading - newHeading) >= compassRotationTolerance){
            // If compass has detected a rotation, pass test and continue
            return true;
        }
        hal.scheduler->delay(interactiveTestLoopDelay);
    }
    return false;
}

static bool _cervello_interactiveSDCard(void){
    // Expect delay based on timeout duration;
    EXPECT_DELAY_MS((int)interactiveTestTimeout);

    // Initialise the logging system
    if(!_initialiseLogger()){
        hal.console->printf("Logging system could not be initialised\n");
        return false;
    }

    // Verify an SD Card has been detected
    if (!logger.CardInserted()){
        hal.console->printf("Could not find SD Card\n");
        return false;
    }

    // Retrieve the current number of logfiles
    const int initialNumLogFiles = logger.get_num_logs();

    // Arm the vehicle to create a new log file
    logger.set_vehicle_armed(true);
    hal.scheduler->delay(sdCardActivityDelay);

    // Write a sample message
    logger.Write_Message("Cervello SD Card Test");
    hal.scheduler->delay(sdCardActivityDelay);

    // Disarm the vehicle to close the log file
    logger.set_vehicle_armed(false);
    logger.StopLogging();
    hal.console->printf("Testing SD Card --- ");
    hal.scheduler->delay(sdCardActivityDelay);

    // Retrieve the new number of logfiles
    const int finalNumLogFiles = logger.get_num_logs();

    // Test pass if the logfile was written
    static bool testResult = finalNumLogFiles > initialNumLogFiles;
    hal.console->printf(kResultStr[testResult]); hal.console->printf("\n");
    return testResult;
}

static bool _cervello_interactiveRAMTRON(void){
    // Initialise the RAMTRON
    _initialiseRAMTRON();

    // Initialise the random number generator
    _initialiseRandomNumberGenerator();

    // Setup variable to track test result
    bool summaryTestResult = true;

    // Test writing 0x00 (0)
    uint8_t testValue = 0x00;
    hal.console->printf("Testing RAMTRON sequential write/read with value of %u --- ", testValue);
    summaryTestResult &= _cervello_interactiveRAMTRON_writeValue(testValue);
    hal.console->printf(kResultStr[summaryTestResult]);

    // Test writing 0xFF (255)
    testValue = 0xFF;
    hal.console->printf("Testing RAMTRON sequential write/read with value of %u --- ", testValue);
    summaryTestResult &= _cervello_interactiveRAMTRON_writeValue(testValue);
    hal.console->printf(kResultStr[summaryTestResult]);

    // Test writing random numbers
    hal.console->printf("Testing RAMTRON sequential write/read with random numbers -- ");
    summaryTestResult &= _cervello_interactiveRAMTRON_writeRandom();
    hal.console->printf(kResultStr[summaryTestResult]);

    // Write all zeros to reset RAMTRON to a known state
    _cervello_interactiveRAMTRON_writeValue(0);
     hal.console->printf("\n");

    return summaryTestResult;
}

static bool _cervello_interactiveRAMTRON_writeValue(uint8_t valueToWrite){
    // Test to write a known value to all addresses in the RAMTRON memory

    // Write test data to RAMTRON
    int ramtronSize = ramtron.get_size();
    EXPECT_DELAY_MS(interactiveTestTimeout);

    for (int i = 0; i < ramtronSize; i++){
        uint8_t bytesWritten = ramtron.write(i, &valueToWrite, sizeof(uint8_t));

        // Verify data was successfully written
        if (bytesWritten==0){
            hal.console->printf("Write failure at index %i --- ",i);
            return false;
        }
    }

    // Read data back from RAMTRON,
    std::vector<uint8_t> readbackData;
    readbackData.reserve(ramtronSize);
    
    for (int j = 0; j < ramtronSize; j++){
        uint8_t bytesRead = ramtron.read(j, &readbackData[j], sizeof(uint8_t));

        // Verify data was successfully read
        if (bytesRead==0){
            hal.console->printf("Read failure at index %i --- ", j);
            return false;
        }
    }

    // Compare the values of the written data to the read data
    for (int l = 0; l < ramtronSize; l++){

        if (valueToWrite != readbackData[l]){
            hal.console->printf("Value mismatch at index %i - Written Value: %u Read Value %u ", l, valueToWrite, readbackData[l]);
            return false;                    
        }
    }

    return true;
}

static bool _cervello_interactiveRAMTRON_writeRandom(void){
    // Test to write a random array to the entirety of the RAMTRON
    
    if (ramtron.get_size()>cervelloRamtronSize){
        hal.console->printf(" This RAMTRON test is currently only applicable for Cervello boards with 16KB memory! --- ");
        return false;
    }

    EXPECT_DELAY_MS(interactiveTestTimeout);

    // Generate random data
    static uint8_t randomArray[cervelloRamtronSize];
    for (int i = 0; i < cervelloRamtronSize; i++){
        randomArray[i] = (uint8_t)rand();
    }

    // Write data to the RAMTRON
    uint32_t bytesWritten = ramtron.write(0, &randomArray[0], cervelloRamtronSize);

    // Verify data was successfully written
    if (bytesWritten==0){
        hal.console->printf(" Random array write failure --- ");
        return false;
    }

    // Read data back from the RAMTRON
    uint8_t readBackData;
    for (int k = 0; k < cervelloRamtronSize; k++){
        uint32_t bytesRead = ramtron.read(k, &readBackData, sizeof(uint8_t));

        // Verify data was successfully written
        if (bytesRead==0){
            hal.console->printf(" Random array read failure at offset %i --- ",k);
            return false;
        }

        // Verify the read data matches the original written data
        if (randomArray[k] != readBackData){
            hal.console->printf("Value mismatch at index %i - Written: %u Read %u ", k, randomArray[k], readBackData);
            return false;        
        }

    }
    return true;
}

// test cases - PPDS Carrier Board
static bool _PPDSCarrier_serialCommunicationTest(int serialDevice1, int serialDevice2, bool enabledHardwareControlFlow){
    // Test verifying communication between two serial devices, using a crosstalk cable

    // Setup serial devices
    std::vector<int> serialDeviceIDs = {serialDevice1, serialDevice2};

    AP_HAL::UARTDriver* SerialDevice[serialDeviceIDs.size()];
    for (int i = 0; i < serialDeviceIDs.size(); i++){
        int deviceID = serialDeviceIDs[i];

        // Retrieve device driver
        SerialDevice[i] = serialManager.get_serial_by_id(deviceID);

        // Begin device/Flush RX & TX buffers
        SerialDevice[i]->begin((uint32_t)UARTbaud);

        // Set flow control
        if (enabledHardwareControlFlow) {
            SerialDevice[i]->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_ENABLE);
        }
        else {
            SerialDevice[i]->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        }
    }
 
    // Generate test message
    uint8_t tx_buffer[] = {12, 34};

    // Write data using Serial Device #1
    SerialDevice[0]->write(tx_buffer, (size_t)sizeof(tx_buffer));
    hal.scheduler->delay(UARTwriteDelay);

    // Verify that bytes exist in the read buffer
    size_t nBytes = SerialDevice[1]->available();
    if (nBytes < 1){
        hal.console->printf("No bytes found in read buffer --- ");
        return false;
    }

    // Read data using Serial Device #2
    std::vector<uint8_t> rx_buffer;
    rx_buffer.reserve(nBytes);
    if (nBytes) {
        while (nBytes-- > 0) {
            uint8_t c = (uint8_t)SerialDevice[1]->read();
            rx_buffer.push_back(c);
         }
    }

    // Verify that the send and received message matches the original test message
    for (int i = 0; i < rx_buffer.size(); i++){
        if (tx_buffer[i] != rx_buffer[i]){
            hal.console->printf("Value mismatch at index %i - Written Value: %u Read Value %u ", i, tx_buffer[i], rx_buffer[i]);
            return false;           
        }
    }
 
    return true;
}

static bool _PPDSCarrier_buzzerTest(void){
    // Test to verify that the Buzzer on the PPDS Carrier Board is functional
      
    if (!notify.buzzer_enabled()){
        hal.console->printf("Buzzer not enabled ");
        return false;
    }
    
    // Tunes defined in ToneAlarm.cpp
    hal.console->printf("Buzzer generating tone --- ");
    AP_Notify::play_tune("MFT100L4>G#6A#6B#4");

    return true;
}

static bool _PPDSCarrier_safetySwitchTest(void){
    // Test to verify that the safety switch on the PPDS Carrier Board is functional

    // Expect delay based on timeout duration;
    EXPECT_DELAY_MS((int)interactiveTestTimeout);

    // Setup test duration
    uint32_t testStartTime = AP_HAL::micros();
    uint32_t testEndTime = testStartTime + (uint32_t)interactiveTestTimeout;

    // Get the current switch state
    int originalSwitchState = hal.util->safety_switch_state();

    // Wait for the user to activate the safety switch
    hal.console->printf("Waiting for switch to be activated --- ");
    while(AP_HAL::micros() < testEndTime){

        // Check if the switch has been actuated
        if (hal.util->safety_switch_state() != originalSwitchState){
            return true;
        }

        hal.scheduler->delay(interactiveTestLoopDelay);
    }
    return false;
}

const struct AP_Param::GroupInfo        GCS_MAVLINK::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
