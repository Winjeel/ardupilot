/*
  Cervello board test
*/
#include "BoardTest.h"

#if APJ_BOARD_ID != 1688
    // TODO: Fix the board ID (currently 11=FMUv4).
    #error This BoardTest is currently only applicable for Cervello boards!
#endif

// Board classes
const AP_HAL::HAL &hal = AP_HAL::get_HAL();
static AP_BoardConfig boardConfig;
static AP_SerialManager serialManager;

// Sensor classes
static AP_Baro barometer;
static Compass compass;
static AP_InertialSensor ins;

// On-board memory classes
AP_RAMTRON ramtron;
AP_Int32 log_bitmask;
AP_Logger logger{log_bitmask};

// Utility classes
static uint32_t timer;
static uint32_t sNow_ms = 0;

// initialisation functions
static void _initialiseCervello(void){
    // function to initialise the Cervello
    
    // initialise serial port
    serialManager.init_console();

    while (!hal.console->is_initialized()) {
        hal.scheduler->delay(100);
    }

    // initialise serial ports
    serialManager.init();

    // initialise Cervello
    boardConfig.init();
    hal.scheduler->delay(1000);

};

static void _initialiseConsole(void) {
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

static void _initialiseLogger(void){
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

void setup()
{
    // initialise Cervello
    _initialiseCervello();
    _initialiseLED();
    
    // initialise test console
    _initialiseConsole();

    // set up timer to count time in microseconds
    timer = AP_HAL::micros();
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

static bool _runAllTests_Cervello_Probe(void){
    // Function to run all probe tests on the Cervello
    bool summaryTestResult = true;
    bool testResult;

    EXPECT_DELAY_MS(probeTestTimeout);

    hal.console->printf("Probing MS5611 (Baro - SPI)\n");
    testResult = _testMS5611_probe();
    (testResult) ? hal.console->printf("PASS\n\n") : hal.console->printf("FAIL\n\n");
    summaryTestResult &= testResult;

    hal.console->printf("Probing ICM20602 (IMU1 - SPI)\n");
    testResult = _testICM20602_probe();
    (testResult) ? hal.console->printf("PASS\n\n") : hal.console->printf("FAIL\n\n");
    summaryTestResult &= testResult;

    hal.console->printf("Probing ICM20948 (IMU2 - SPI)\n");
    testResult = _testICM20948_imu_probe();
    (testResult) ? hal.console->printf("PASS\n\n") : hal.console->printf("FAIL\n\n");
    summaryTestResult &= testResult;

    hal.console->printf("Probing ICM20948 (Compass - SPI)\n");
    testResult = _testICM20948_mag_probe();
    (testResult) ? hal.console->printf("PASS\n\n") : hal.console->printf("FAIL\n\n");
    summaryTestResult &= testResult;

    hal.console->printf("Probing IST8308 (Compass - I2C)\n");
    testResult = _testIST8308_probe();
    (testResult) ? hal.console->printf("PASS\n\n") : hal.console->printf("FAIL\n\n");
    summaryTestResult &= testResult;

    hal.console->printf("Probing RAMTRON\n");
    testResult = _testRamtron_probe();
    (testResult) ? hal.console->printf("PASS\n\n") : hal.console->printf("FAIL\n\n");
    summaryTestResult &= testResult;

    hal.console->printf("WARNING - Cervello requires reset to cleanup dirty driver state\n\n");
    return summaryTestResult;
}

static bool _runAllTests_Cervello_Interactive(void){
    // function to run all interrogation tests on the Cervello
    bool summaryTestResult = true;
    
    // run the Accelerometer and Gyro tests
    summaryTestResult &= _interactiveTest_Accel();
    summaryTestResult &= _interactiveTest_Gyro();

    // run the compass tests
    summaryTestResult &= _interactiveTest_Compass();

    // run the barometer tests
    summaryTestResult &= _interactiveTest_Barometer();

    // run the SD Card tests
    summaryTestResult &= _interactiveTest_SDCard();

    // run the RAMTRON tests
    summaryTestResult &= _interactiveTest_RAMTRON();

    hal.console->printf("WARNING - Cervello requires reset to cleanup dirty driver state\n\n");
    return summaryTestResult;
}

// test cases - cervello probe
static bool _testMS5611_probe(void) { // Baro 1, SPI
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

static bool _testICM20602_probe(void) { // IMU 1, SPI
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

static bool _testICM20948_imu_probe(void) { // IMU 2, SPI
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

static bool _testICM20948_mag_probe(void) { // Compass 1, SPI
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

static bool _testIST8308_probe(void) { // Compass 2, I2C
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

static bool _testRamtron_probe(void){ // RAMTRON
    return ramtron.init();
}

static bool _interactiveTest_Accel(void){
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

        // Loop through each accelerometer
        for (uint8_t j = 0; j < ins.get_accel_count(); j++) {
            const float * ptr_to_accel;

            switch(i){
                case 0: // X Axis test
                    if (j==0) {hal.console->printf("Orient the board with the X axis facing down\n");};
                    ptr_to_accel = &ins.get_accel(j).x;
                    break;

                case 1: // Y Axis test
                    if (j==0) {hal.console->printf("Orient the board with the Y axis facing down\n");};
                    ptr_to_accel = &ins.get_accel(j).y;
                    break;

                default: // Z Axis test
                    if (j==0) {hal.console->printf("Orient the board with the Z axis facing down\n");};
                    ptr_to_accel = &ins.get_accel(j).z;
                    break;
            }

            hal.console->printf("Testing accelerometer %i --- ",j);
            bool testResult =_interactiveTest_Accel_SingleAxis(ptr_to_accel);
            (testResult) ? hal.console->printf("PASS\n") : hal.console->printf("FAIL\n");
            summaryTestResult &= testResult;

        } // Accelerometer loop
        hal.console->printf("\n");
    } // Axis loop

    return summaryTestResult;
}

static bool _interactiveTest_Gyro(void){
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

        // Loop through each gyro
        for (uint8_t j = 0; j < ins.get_gyro_count(); j++) {
            const float * ptr_to_gyro;

            switch(i){
                case 0: // X Axis test
                    if (j==0) {hal.console->printf("Rotate the board clockwise around the positive X axis\n");};
                    ptr_to_gyro = &ins.get_gyro(j).x;
                    break;

                case 1: // Y Axis test
                    if (j==0) {hal.console->printf("Rotate the board clockwise around the positive Y axis\n");};
                    ptr_to_gyro = &ins.get_gyro(j).y;
                    break;

                default: // Z Axis test
                    if (j==0) {hal.console->printf("Rotate the board clockwise around the positive Z axis\n");};
                    ptr_to_gyro = &ins.get_gyro(j).z;
                    break;
            }

            hal.console->printf("Testing gyro %i --- ",j);
            bool testResult =_interactiveTest_Gyro_SingleAxis(ptr_to_gyro);
            (testResult) ? hal.console->printf("PASS\n") : hal.console->printf("FAIL\n");
            summaryTestResult &= testResult;

        } // Gyro loop
        hal.console->printf("\n");
    } // Axis loop

    return summaryTestResult;
}

static bool _interactiveTest_Accel_SingleAxis(const float* accelSensorPtr){
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
        float accelData = *accelSensorPtr;
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

static bool _interactiveTest_Gyro_SingleAxis(const float* gyroSensorPtr){
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
        float gyroData = *gyroSensorPtr;
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

static bool _interactiveTest_Barometer(void){
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
    hal.console->printf("Testing barometer temperature within range %.1fC to %.1fC -- ", baro_temp_expectedMin, baro_temp_expectedMax);
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
    hal.console->printf("Testing barometer pressure within range %.1fPa to %.1fPa -- ", baro_pressure_expectedMin, baro_pressure_expectedMax);
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

static bool _interactiveTest_Compass(void){
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
    hal.console->printf("Orient the board with the X axis facing towards magnetic north\n");
    for (uint8_t j = 0; j < compass.get_count(); j++) {

        hal.console->printf("Testing compass %i --- ",j);
        bool testResult =_interactiveTest_Compass_SingleHeading(j);
        (testResult) ? hal.console->printf("PASS\n") : hal.console->printf("FAIL\n");
        summaryTestResult &= testResult;

    } // Compass loop
    hal.console->printf("\n");

    return summaryTestResult;
}

static bool _interactiveTest_Compass_SingleHeading(const int i){
    // Expect delay based on timeout duration;
    EXPECT_DELAY_MS((int)interactiveTestTimeout);

    // Setup test duration
    uint32_t testStartTime = AP_HAL::micros();
    uint32_t testEndTime = testStartTime + (uint32_t)interactiveTestTimeout;

    // Setup variable to track running average
    float runningAverage = 180; // initialise to nonzero

    // Poll the compass data
    while(AP_HAL::micros() < testEndTime){

        // Update compass and retrieve data
        compass.read();

        // calculate the heading offset from magnetic north
        Matrix3f dcm_matrix;
        dcm_matrix.from_euler(0, 0, 0); // roll pitch yaw 0
        float heading = ToDeg(compass.calculate_heading(dcm_matrix, i));

        // Update the running average
        runningAverage = _approxRunningAverage(runningAverage, abs(heading));

        // Check if compass is aligned with magnetic north
        if (_checkCompassAlignment(runningAverage)){
            // If compass is aligned, pass test and continue
            return true;
        }
        hal.scheduler->delay(interactiveTestLoopDelay);
    }
    return false;
}

static bool _interactiveTest_SDCard(void){
    // Expect delay based on timeout duration;
    EXPECT_DELAY_MS((int)interactiveTestTimeout);

    // Initialise the logging system
    _initialiseLogger();

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
    (testResult) ? hal.console->printf("PASS\n\n") : hal.console->printf("FAIL\n\n");
    return testResult;
}

static bool _interactiveTest_RAMTRON(void){
    // Initialise the RAMTRON
    _initialiseRAMTRON();

    // Initialise the random number generator
    _initialiseRandomNumberGenerator();

    // Setup variable to track test result
    bool summaryTestResult = true;

    // Test writing min uint8_t (zero) sequentually
    uint8_t testValue = 0;
    hal.console->printf("Testing RAMTRON sequential write/read with value of %u --- ", testValue);
    summaryTestResult &= _interactiveTest_RAMTRON_writeValue(testValue, false, false);
    (summaryTestResult)? hal.console->printf("PASS\n") : hal.console->printf("FAIL\n");

    // Test writing max uint8_t (255) randomly
    testValue = 256-1;
    hal.console->printf("Testing RAMTRON random write/read with value of %u --- ", testValue);
    summaryTestResult &= _interactiveTest_RAMTRON_writeValue(testValue, true, false);
    (summaryTestResult)? hal.console->printf("PASS\n") : hal.console->printf("FAIL\n");

    // Test writing random numbers in random order
    hal.console->printf("Testing RAMTRON random write/read with random numbers -- ");
    summaryTestResult &= _interactiveTest_RAMTRON_writeValue(0, true, true);
    (summaryTestResult)? hal.console->printf("PASS\n") : hal.console->printf("FAIL\n");

    // Write all zeros to reset RAMTRON to a known state
    _interactiveTest_RAMTRON_writeValue(0, false, false);

    return summaryTestResult;
}

static bool _interactiveTest_RAMTRON_writeValue(uint8_t valueToWrite, bool randomWrite, bool writeRandomValues){

    // Generate the offsets in RAMTRON memory (either sequential or random) to write data to
    int ramtronSize = ramtron.get_size();
    std::vector<int> writeIndexes = _createIndexArray(ramtronSize, randomWrite);

    // Generate test data
    std::vector<uint8_t> testData;
    testData.reserve(ramtronSize);
    for (int i = 0; i < ramtronSize; i++){

        if(writeRandomValues){
            testData.push_back((uint8_t)rand());
        }
        else {
            testData.push_back(valueToWrite);
        }       
    }

    // Write test data to RAMTRON
    EXPECT_DELAY_MS(interactiveTestTimeout);

    for (int j = 0; j < ramtronSize; j++){
        uint8_t writeSuccess = ramtron.write(writeIndexes[j], &testData[j], sizeof(uint8_t));

        // Verify data was successfully written
        if (writeSuccess==0){
            hal.console->printf("Write failure at index %u --- ",(uint)writeIndexes[j]);
            return false;
        }
    }

    // Read data back from RAMTRON,
    std::vector<uint8_t> readbackData;
    readbackData.reserve(ramtronSize);
    
    for (int k = 0; k < ramtronSize; k++){
        uint8_t readSuccess = ramtron.read(writeIndexes[k], &readbackData[k], sizeof(uint8_t));

        // Verify data was successfully read
        if (readSuccess==0){
            hal.console->printf("Read failure at index %i --- ", writeIndexes[k]);
            return false;
        }
    }

    // Compare the values of the written data to the read data
    for (int l = 0; l < ramtronSize; l++){

        //hal.console->printf("index %i - Written Value: %u Read Value %u \n", l, testData[l], readbackData[l]);

        if (testData[l] != readbackData[l]){
            hal.console->printf("Value mismatch at index %i - Written Value: %u Read Value %u ", l, testData[l], readbackData[l]);
            return false;                    
        }
    }

    return true;
}

const struct AP_Param::GroupInfo        GCS_MAVLINK::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();
