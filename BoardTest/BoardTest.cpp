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

// Utility classes
static uint32_t timer;
//static uint8_t counter;
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
    // function to initialise the barometer
    barometer.init();
    barometer.calibrate();
};

static void _initialiseCompass(void){
    // function to initialise the compass
    compass.init();

    for (uint8_t i = 0; i < compass.get_count(); i++) {
        compass.set_and_save_offsets(i, Vector3f(0, 0, 0));
    }

    compass.set_declination(ToRad(0.0f));
};

static void _initialiseIMU(void){
    // function to initialise the IMU
    ins.init(100);
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

    hal.console->printf("%65s\n", AP::fwversion().fw_string);
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
    
    // initialise sensors
    _initialiseBarometer();
    _initialiseCompass();
    _initialiseIMU();

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

// test cases
static bool _testMS5611_interrogate(void) { // Test 1 - Baro 1, SPI
    bool result = false;
    static AP_Baro_Backend *sMS5611_backend = nullptr;

    if (!sMS5611_backend) {
        // Probe the parometer backend
        sMS5611_backend = HAL_BARO_1_PROBE(barometer);
    }

    if (sMS5611_backend) {
        // If the barometer backend exists, test pass
        result = true;
    }

    hal.console->printf("Warning - board may require reset to ensure clean test state");
    return result;
}

static bool _testICM20602_interrogate(void) { // Test 2 - IMU 1, SPI
    bool result = false;
    static AP_InertialSensor_Backend *sICM20602_backend = nullptr;

    if (!sICM20602_backend) {
        // Probe the IMU backend
        sICM20602_backend = HAL_INS_1_PROBE(ins);
    }

    if (sICM20602_backend) {
        // If the IMU backend exists, test pass
        result = true;
    }

    hal.console->printf("Warning - board may require reset to ensure clean test state");
    return result;
}

static bool _testICM20948_imu_interrogate(void) { // Test 3 - IMU 2, SPI
    bool result = false;
    static AP_InertialSensor_Backend *sICM20948_imu_backend = nullptr;

    if (!sICM20948_imu_backend) {
        // Probe the IMU backend
        sICM20948_imu_backend = HAL_INS_2_PROBE(ins);
    }

    if (sICM20948_imu_backend) {
        // If the IMU backend exists, test pass
        result = true;
    }

    hal.console->printf("Warning - board may require reset to ensure clean test state");
    return result;
}

static bool _testICM20948_mag_interrogate(void) { // Test 4 - Compass 1, SPI
    bool result = false;
    static AP_Compass_Backend *sICM20948_mag_backend = nullptr;

    if (!sICM20948_mag_backend) {
        // Probe the compass backend
        sICM20948_mag_backend = HAL_MAG_1_PROBE;
    }

    if (sICM20948_mag_backend) {
        // If the compass backend exists, test pass
        result = true;
    }

    hal.console->printf("Warning - board may require reset to ensure clean test state");
    return result;
}

static bool _testIST8308_interrogate(void) { // Test 5 - Compass 2, I2C
    bool result = false;
    static AP_Compass_Backend * sIST8308_backend = nullptr;

    if (!sIST8308_backend) {
        // Probe the compass backend
        sIST8308_backend = HAL_MAG_2_PROBE;
    }

    if (sIST8308_backend) {
        // If the compass backend exists, test pass
        result = true;
    }

    hal.console->printf("Warning - board may require reset to ensure clean test state");
    return result;
}

static bool _testBarometer_sensorData(void){

    if (!barometer.all_healthy()){
        hal.console->print("Barometer not healthy\n");
        return false;
    }

    barometer.accumulate();
    barometer.update();

    //output barometer readings to console
    hal.console->printf("Pressure: %.2f Pa\n"
                        "Temperature: %.2f degC\n"
                        "Relative Altitude: %.2f m\n",
                        (double)barometer.get_pressure(),
                        (double)barometer.get_temperature(),
                        (double)barometer.get_altitude());

    return barometer.get_pressure()>0;
}

static bool _testCompass_sensorData(void){
    bool healthStatus = true;
    bool magStatus = true;

    // Check all compass sensors are healthy
    for (uint8_t i = 0; i < compass.get_count(); i++) {
        healthStatus &= compass.healthy(i);
    }

    if (!healthStatus){
        hal.console->print("Compass not healthy\n");
        return false;
    }

    // Read measurements from the compass sensors
    compass.read();

    for (uint8_t i = 0; i < compass.get_count(); i++) {
        const Vector3f &mag = compass.get_field(i);
        hal.console->printf("Compass %i - MagX: %.2f MagY: %.2f MagZ: %.2f\n", i, mag.x, mag.y, mag.z);
        magStatus &= mag.length() > 0;
    }

    return magStatus;
}

static bool _testINS_sensorData_accel(void){
    bool gravityCheck = true;

    // Check all accelerometers sensors are healthy
    if (!ins.get_accel_health_all()){
        hal.console->print("Accelerometers not healthy\n");
        return false;
    }

    // Update the accelerometers
    ins.update();

    // Print accelerometer data
    for (uint8_t i = 0; i < ins.get_accel_count(); i++) {
        const Vector3f &acc = ins.get_accel(i);
        hal.console->printf("Accelerometer %i - AccX: %.2f AccY: %.2f AccZ: %.2f\n", i, acc.x, acc.y, acc.z);

        gravityCheck &= abs((double)acc.x) < accelTol;
        gravityCheck &= abs((double)acc.y) < accelTol;
        gravityCheck &= (abs((double)acc.z) - 9.81) < accelTol;
    }

    return gravityCheck;
}

static bool _testINS_sensorData_gyro(void){
    bool rotationCheck = true;    

    // Check all gyros sensors are healthy
    if (!ins.get_gyro_health_all()){
        hal.console->print("Gyros not healthy\n");
        return false;
    }

    // Update the gyros
    ins.update();

    // Print gyro data
    for (uint8_t i = 0; i < ins.get_gyro_count(); i++) {
        const Vector3f &rot = ins.get_gyro(i);
        hal.console->printf("Gyro %i - RotX: %.2f RotY: %.2f RotZ: %.2f\n", i, rot.x, rot.y, rot.z);

        rotationCheck &= abs((double)rot.x) < gyroTol;
        rotationCheck &= abs((double)rot.y) < gyroTol;
        rotationCheck &= abs((double)rot.z) < gyroTol;
    }
    return rotationCheck;
}

static bool _testINS_accel_xAxis(void){

    // Verify that the INS sensors exist before continuing
    if (ins.get_accel_count() < 1){
        hal.console->printf("No INS sensors found\n");
        return false;
    }

    // Setup variable to track test result
    std::vector<bool> testResults(ins.get_accel_count());
    bool summaryTestResult = true;

    // Expect delay based on timeout duration * number of sensors
    EXPECT_DELAY_MS((int)interactiveTestTimeout * (int)ins.get_accel_count());
    hal.console->printf("Orient the board with the X axis facing down\n\n");

    // Test each accelerometer
    for (uint8_t i = 0; i < ins.get_accel_count(); i++) {
        hal.console->printf("Testing accelerometer %i\n",i);
        testResults[i] = false;

        // Setup test duration
        uint32_t testStartTime = AP_HAL::micros();
        uint32_t testEndTime = testStartTime + (uint32_t)interactiveTestTimeout;

        // Setup variable to track running average
        float runningAverage = 0;

        // Poll the accelerometer data
        while(AP_HAL::micros() < testEndTime){

            // Update accelerometer and retrieve data
            ins.update();
            const Vector3f &acc = ins.get_accel(i);

            // Update the running average
            runningAverage = _approxRunningAverage(runningAverage, acc.x);

            // Check if accelerometer is aligned with gravity
            if (_checkGravityAcceleration(runningAverage)){
                // If accelerometer is aligned, pass test and continue to the next accelerometer
                testResults[i] = true;
                break;
            }
            hal.scheduler->delay(5);
        }

        (testResults[i]) ? hal.console->printf("Accelerometer %i pass\n\n",i) : hal.console->printf("Accelerometer %i fail\n\n",i);

    }

    // Return overall test result across all accelerometers
    for (int j = 0; j < testResults.size(); j++){
        summaryTestResult &= testResults[j];
    }

    return summaryTestResult;
}

static bool _testINS_gyro_xAxis(void){

    // Verify that the INS sensors exist before continuing
    if (ins.get_gyro_count() < 1){
        hal.console->printf("No INS sensors found\n");
        return false;
    }

    // Setup variable to track test result
    std::vector<bool> testResults(ins.get_accel_count());
    bool summaryTestResult = true;

    // Expect delay based on timeout duration * number of sensors
    EXPECT_DELAY_MS((int)interactiveTestTimeout * (int)ins.get_accel_count());
    hal.console->printf("Rotate the board clockwise around the positive X axis\n\n");

    // Test each gyro
    for (uint8_t i = 0; i < ins.get_gyro_count(); i++) {
        hal.console->printf("Testing gyro %i\n",i);
        testResults[i] = false;

        // Setup test duration
        uint32_t testStartTime = AP_HAL::micros();
        uint32_t testEndTime = testStartTime + (uint32_t)interactiveTestTimeout;

        // Setup variable to track running average
        float runningAverage = 0;

        // Poll the gyro data
        while(AP_HAL::micros() < testEndTime){

            // Update accelerometer and retrieve data
            ins.update();
            const Vector3f &rot = ins.get_gyro(i);

            // Update the running average
            runningAverage = _approxRunningAverage(runningAverage, rot.x);

            // Check if the gyro senses positive rotation
            if (_checkRotation(runningAverage)){
                // If positive rotation, pass test and continue to the next gyro
                testResults[i] = true;
                break;
            }
            hal.scheduler->delay(5);
        }

        (testResults[i]) ? hal.console->printf("Gyro %i pass\n\n",i) : hal.console->printf("Gyro %i fail\n\n",i);

    }

    // Return overall test result across all gyros
    for (int j = 0; j < testResults.size(); j++){
        summaryTestResult &= testResults[j];
    }

    return summaryTestResult;
}

bool _checkGravityAcceleration(float acceleration){
    // This function checks for the reactive force against gravity
    float accelDelta = acceleration + GRAVITY_MSS;

    return abs(accelDelta) < accelTol;
}

bool _checkRotation(float rotation){
    // Function to check for positive rotation from the current gyro axis
    return rotation > gyroTol;
}

float _approxRunningAverage(float average, float newSample){
    average -= average / runningAverageSamples;
    average += newSample / runningAverageSamples;
    return average;
    // https://stackoverflow.com/questions/12636613/how-to-calculate-moving-average-without-keeping-the-count-and-data-total

}

const struct AP_Param::GroupInfo        GCS_MAVLINK::var_info[] = {
    AP_GROUPEND
};
GCS_Dummy _gcs;

AP_HAL_MAIN();





static bool _call_generic_AccelTest(void){

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

                case 2: // Z Axis test
                    if (j==0) {hal.console->printf("Orient the board with the Z axis facing down\n");};
                    ptr_to_accel = &ins.get_accel(j).z;
                    break;
            }

            hal.console->printf("Testing accelerometer %i --- ",j);
            bool testResult =_generic_AccelTest(ptr_to_accel);
            (testResult) ? hal.console->printf("PASS\n") : hal.console->printf("FAIL\n");
            summaryTestResult &= testResult;

        } // Accelerometer loop
        hal.console->printf("\n");
    } // Axis loop

    return summaryTestResult;

}

static bool _generic_AccelTest(const float* accelSensorPtr){
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

//        hal.console->printf("Running average %.2f\n",runningAverage);

        // Check if accelerometer is aligned with gravity
        if (_checkGravityAcceleration(runningAverage)){
            // If accelerometer is aligned, pass test and continue
            return true;
        }
        hal.scheduler->delay(5);
    }
    return false;
}