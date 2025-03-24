#include "BLECStringCharacteristic.h"
#include "EString.h"
#include <math.h>
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "SparkFun_VL53L1X.h"

#define SERIAL_PORT Serial
#define AD0_VAL 1 // The value of the last bit of the I2C address, on the SparkFun 9DoF IMU breakout the default is 1
#define blinkPin LED_BUILTIN

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "ce2aed5f-e2fa-4c97-8e64-40a95acc6e08"

#define BLE_UUID_RX_STRING "9750f60b-9c9c-4158-b620-02ec9521cd99"

#define BLE_UUID_TX_FLOAT "27616294-3063-4ecc-b60b-3470ddef2938"
#define BLE_UUID_TX_STRING "f235a225-6735-4d73-94cb-ee5dfce9ba83"

//////////// BLE UUIDs ////////////

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);

BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);

BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

// ToF sensor data arrays
const int num_data_msgs = 1000;
float acc_pitch_raw[num_data_msgs];
float acc_pitch_lpf[num_data_msgs];
float acc_roll_raw[num_data_msgs];
float acc_roll_lpf[num_data_msgs];

float gyro_pitch_raw[num_data_msgs];
float gyro_roll_raw[num_data_msgs];
float gyro_yaw_raw[num_data_msgs];

float comp_pitch[num_data_msgs];
float comp_roll[num_data_msgs];
float comp_yaw[num_data_msgs];

float distance1_data[num_data_msgs];
float distance2_data[num_data_msgs];
float pwm_data[num_data_msgs];

// time array
int times[num_data_msgs];

int num_vars_data_collection;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
float temps[num_data_msgs]; // temporary array

// pwm/driving tuning vars
float calibration_factor = 0.86;
// int pwm = 255;

// PID vars
bool pid_on = false;
int pid_i; // index
float start_time;
float end_time;
float current_time;
int error_sum = 0;
float target = 304.8;
int max_speed = 200;

float Kp = 0.05;
float Ki = 0.0001;
float Kd;

float Kp_ori = 4.0;
float Ki_ori = 0.0001;
float Kd_ori;

// PID orientation vars
bool pid_ori_on = false;
int pid_ori_i;

double error_ori;
double prev_error_ori;
double error_sum_ori;
float yaw;
int pid_speed;
float prev_time;
double current_angle;

float target_angle = 0;

int pid_times[num_data_msgs];
double pid_current_angles[num_data_msgs];
double pid_target_angles[num_data_msgs];
int pid_speeds[num_data_msgs];
double pid_err[num_data_msgs];
double pid_ps[num_data_msgs];
double pid_is[num_data_msgs];
double pid_ds[num_data_msgs];

#define XSHUT 8
#define ADDRESS 0x30

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2(Wire, XSHUT);

//////////// Global Variables ////////////

enum CommandTypes
{
    START_PID,
    STOP_PID,
    GET_PID_DATA,
    GET_PITCH_DATA,
    GET_ROLL_DATA,
    GET_YAW_DATA,
    GET_IMU_DATA,
    GET_TOF_DATA,
    START_PID_ORI,
    STOP_PID_ORI,
    GET_PID_DATA_ORI,
};

void set_up_tof()
{
    // set up tof sensors
    digitalWrite(XSHUT, LOW);
    distanceSensor1.setI2CAddress(ADDRESS);
    Serial.print("Distance Sensor 1 Address: 0x");
    Serial.println(distanceSensor1.getI2CAddress(), HEX);

    if (distanceSensor1.begin() != 0) // Begin returns 0 on a good init
    {
        Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
        while (1)
            ;
    }

    digitalWrite(XSHUT, HIGH);
    Serial.print("Distance Sensor 2 Address: 0x");
    Serial.println(distanceSensor2.getI2CAddress(), HEX);

    if (distanceSensor2.begin() != 0) // Begin returns 0 on a good init
    {
        Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
        while (1)
            ;
    }
    Serial.println("Sensors 1 and 2 online!");

    distanceSensor1.setDistanceModeShort();
    distanceSensor2.setDistanceModeShort();
}

void handle_command()
{
    // Set the command string from the characteristic value
    robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                             rx_characteristic_string.valueLength());

    bool success;
    int cmd_type = -1;

    // Get robot command type (an integer)
    /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
     * since it uses strtok internally (refer RobotCommand.h and
     * https://www.cplusplus.com/reference/cstring/strtok/)
     */
    success = robot_cmd.get_command_type(cmd_type);

    // Check if the last tokenization was successful and return if failed
    if (!success)
    {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type)
    {

    case START_PID:
    {
        set_up_tof();

        pid_i = 0;
        start_time = (float)millis();
        error_sum = 0;
        distanceSensor1.startRanging();

        pid_on = true;
        break;
    }

    case STOP_PID:
    {
        pid_on = false;
        drive(0, 0);
        num_vars_data_collection = pid_i;
        break;
    }

    case GET_PID_DATA:
    {
        for (int i = 0; i < num_vars_data_collection; i++)
        {
            tx_estring_value.clear();
            tx_estring_value.append(times[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(distance1_data[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(pwm_data[i]);
            Serial.println(tx_estring_value.c_str());
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }
        break;
    }

    case GET_PITCH_DATA:
    {
        float pitch_g = 0, dt = 0;
        unsigned long last_time = millis();
        // collect pitch data
        for (int i = 0; i < num_data_msgs; i++)
        {
            if (myICM.dataReady())
            {
                myICM.getAGMT();
                // accelerometer data
                float pitch_a = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
                acc_pitch_raw[i] = pitch_a;

                // gyro data
                dt = (millis() - last_time) / 1000.;
                last_time = millis();
                pitch_g = pitch_g + myICM.gyrY() * dt;
                gyro_pitch_raw[i] = pitch_g;

                // time
                times[i] = (int)millis();
            }
        }

        // dt = 1/(sampling rate), RC = 1/(2*pi*cutoff freq), and alpha=dt/(dt+RC)
        const float alpha = 0.0735;

        acc_pitch_lpf[0] = acc_pitch_raw[0];
        comp_pitch[0] = (1 - alpha) * gyro_pitch_raw[0] + alpha * acc_pitch_raw[0];

        for (int n = 1; n < num_data_msgs; n++)
        {
            float pitch_raw_curr = acc_pitch_raw[n];
            acc_pitch_lpf[n] = alpha * pitch_raw_curr + (1 - alpha) * acc_pitch_lpf[n - 1];
            acc_pitch_lpf[n - 1] = acc_pitch_lpf[n];

            comp_pitch[n] = (1 - alpha) * gyro_pitch_raw[n] + alpha * acc_pitch_lpf[n];
        }

        // send pitch data from the pitch array to python
        for (int i = 0; i < num_data_msgs; i++)
        {
            tx_estring_value.clear();
            tx_estring_value.append(times[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(acc_pitch_raw[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(acc_pitch_lpf[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(gyro_pitch_raw[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(comp_pitch[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }

        break;
    }

    case GET_ROLL_DATA:
    {
        float roll_g = 0, dt = 0;
        unsigned long last_time = millis();
        // collect roll data
        for (int i = 0; i < num_data_msgs; i++)
        {
            if (myICM.dataReady())
            {
                myICM.getAGMT();
                // acc data
                float roll_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
                acc_roll_raw[i] = roll_a;

                // gyro data
                dt = (millis() - last_time) / 1000.;
                last_time = millis();
                roll_g = roll_g + myICM.gyrX() * dt;
                gyro_roll_raw[i] = roll_g;

                // time
                times[i] = (int)millis();
            }
        }

        // dt = 1/(sampling rate), RC = 1/(2*pi*cutoff freq), and alpha=dt/(dt+RC)
        const float alpha = 0.0735;

        acc_roll_lpf[0] = acc_roll_raw[0];
        comp_roll[0] = (1 - alpha) * gyro_roll_raw[0] + alpha * acc_roll_raw[0];

        for (int n = 1; n < num_data_msgs; n++)
        {
            float roll_raw_curr = acc_roll_raw[n];
            acc_roll_lpf[n] = alpha * roll_raw_curr + (1 - alpha) * acc_roll_lpf[n - 1];
            acc_roll_lpf[n - 1] = acc_roll_lpf[n];

            comp_roll[n] = (1 - alpha) * gyro_roll_raw[n] + alpha * acc_roll_lpf[n];
        }

        // send roll data from the roll array to python
        for (int i = 0; i < num_data_msgs; i++)
        {
            tx_estring_value.clear();
            tx_estring_value.append(times[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(acc_roll_raw[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(acc_roll_lpf[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(gyro_roll_raw[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(comp_roll[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }

        break;
    }

    case GET_YAW_DATA:
    {
        float yaw_g = 0, dt = 0;
        unsigned long last_time = millis();
        const float alpha = 0.0735;
        // collect roll data
        for (int i = 0; i < num_data_msgs; i++)
        {
            if (myICM.dataReady())
            {
                myICM.getAGMT();
                // gyro data
                dt = (millis() - last_time) / 1000.;
                last_time = millis();
                yaw_g = yaw_g + myICM.gyrZ() * dt;
                gyro_yaw_raw[i] = yaw_g;
                comp_yaw[i] = (1 - alpha) * gyro_yaw_raw[i];

                // time
                times[i] = (int)millis();
            }
        }

        // send roll data from the roll array to python
        for (int i = 0; i < num_data_msgs; i++)
        {
            tx_estring_value.clear();
            tx_estring_value.append(times[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(gyro_yaw_raw[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(comp_yaw[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }

        break;
    }

    case GET_IMU_DATA:
    {
        float yaw_g = 0, roll_g = 0, pitch_g = 0, dt = 0;
        unsigned long last_time = millis();
        const float alpha = 0.0735;
        // collect roll data
        for (int i = 0; i < num_data_msgs; i++)
        {
            if (myICM.dataReady())
            {
                myICM.getAGMT();
                // acc roll data
                float roll_a = atan2(myICM.accX(), myICM.accZ()) * 180 / M_PI;
                acc_roll_raw[i] = roll_a;

                // gyro roll data
                dt = (millis() - last_time) / 1000.;
                last_time = millis();
                roll_g = roll_g + myICM.gyrX() * dt;
                gyro_roll_raw[i] = roll_g;

                // accelerometer data
                float pitch_a = atan2(myICM.accY(), myICM.accZ()) * 180 / M_PI;
                acc_pitch_raw[i] = pitch_a;

                // gyro data
                dt = (millis() - last_time) / 1000.;
                last_time = millis();
                pitch_g = pitch_g + myICM.gyrY() * dt;
                gyro_pitch_raw[i] = pitch_g;

                // gyro data
                dt = (millis() - last_time) / 1000.;
                last_time = millis();
                yaw_g = yaw_g + myICM.gyrZ() * dt;
                gyro_yaw_raw[i] = yaw_g;
                comp_yaw[i] = (1 - alpha) * gyro_yaw_raw[i];

                // time
                times[i] = (int)millis();
            }
        }

        // low pass filters

        acc_pitch_lpf[0] = acc_pitch_raw[0];
        comp_pitch[0] = (1 - alpha) * gyro_pitch_raw[0] + alpha * acc_pitch_raw[0];
        acc_roll_lpf[0] = acc_roll_raw[0];
        comp_roll[0] = (1 - alpha) * gyro_roll_raw[0] + alpha * acc_roll_raw[0];

        for (int n = 1; n < num_data_msgs; n++)
        {
            // lpf pitch
            float pitch_raw_curr = acc_pitch_raw[n];
            acc_pitch_lpf[n] = alpha * pitch_raw_curr + (1 - alpha) * acc_pitch_lpf[n - 1];
            acc_pitch_lpf[n - 1] = acc_pitch_lpf[n];

            comp_pitch[n] = (1 - alpha) * gyro_pitch_raw[n] + alpha * acc_pitch_lpf[n];

            // lpf roll
            float roll_raw_curr = acc_roll_raw[n];
            acc_roll_lpf[n] = alpha * roll_raw_curr + (1 - alpha) * acc_roll_lpf[n - 1];
            acc_roll_lpf[n - 1] = acc_roll_lpf[n];

            comp_roll[n] = (1 - alpha) * gyro_roll_raw[n] + alpha * acc_roll_lpf[n];
        }

        // send roll data from the roll array to python
        for (int i = 0; i < num_data_msgs; i++)
        {
            tx_estring_value.clear();
            tx_estring_value.append(times[i]);
            tx_estring_value.append(" | ");
            tx_estring_value.append(acc_pitch_raw[i]);
            tx_estring_value.append(" | ");
            tx_estring_value.append(acc_pitch_lpf[i]);
            tx_estring_value.append(" | ");
            tx_estring_value.append(acc_roll_raw[i]);
            tx_estring_value.append(" | ");
            tx_estring_value.append(acc_roll_lpf[i]);
            tx_estring_value.append(" | ");
            tx_estring_value.append(gyro_pitch_raw[i]);
            tx_estring_value.append(" | ");
            tx_estring_value.append(gyro_roll_raw[i]);
            tx_estring_value.append(" | ");
            tx_estring_value.append(gyro_yaw_raw[i]);
            tx_estring_value.append(" | ");
            tx_estring_value.append(comp_pitch[i]);
            tx_estring_value.append(" | ");
            tx_estring_value.append(comp_roll[i]);
            tx_estring_value.append(" | ");
            tx_estring_value.append(comp_yaw[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }

        break;
    }

    case GET_TOF_DATA:
    {

        Serial.println("VL53L1X Qwiic Test");

        // set up tof sensors
        digitalWrite(XSHUT, LOW);
        distanceSensor1.setI2CAddress(ADDRESS);
        Serial.print("Distance Sensor 1 Address: 0x");
        Serial.println(distanceSensor1.getI2CAddress(), HEX);

        if (distanceSensor1.begin() != 0) // Begin returns 0 on a good init
        {
            Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
            while (1)
                ;
        }

        digitalWrite(XSHUT, HIGH);
        Serial.print("Distance Sensor 2 Address: 0x");
        Serial.println(distanceSensor2.getI2CAddress(), HEX);

        if (distanceSensor2.begin() != 0) // Begin returns 0 on a good init
        {
            Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
            while (1)
                ;
        }
        Serial.println("Sensors 1 and 2 online!");

        distanceSensor1.setDistanceModeShort();
        distanceSensor2.setDistanceModeShort();

        distanceSensor1.startRanging();
        distanceSensor2.startRanging();

        int st = millis();

        for (int i = 0; i < num_data_msgs; i++)
        {

            while (!distanceSensor1.checkForDataReady())
            {
                delay(1);
            }
            distance1_data[i] = distanceSensor1.getDistance();
            // Serial.print("Distance 1: ");
            // Serial.print(distance1_data[i]);

            while (!distanceSensor2.checkForDataReady())
            {
                delay(1);
            }
            distance2_data[i] = distanceSensor2.getDistance();
            // Serial.print(" | Distance 2: ");
            // Serial.println(distance2_data[i]);

            times[i] = millis();
        }

        int et = millis();

        distanceSensor1.clearInterrupt();
        distanceSensor1.stopRanging();
        distanceSensor2.clearInterrupt();
        distanceSensor2.stopRanging();

        // Serial.print("Time for 500 sensor readings: ");
        // Serial.println(et-st);

        // st = millis();

        // for(int i = 0; i < num_data_msgs; i++)
        // {
        //   continue;
        // }

        // et = millis();

        // Serial.print("Time for 500 iterations of empty loop: ");
        // Serial.println(et-st);

        for (int i = 0; i < num_data_msgs; i++)
        {
            tx_estring_value.clear();
            tx_estring_value.append(times[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(distance1_data[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(distance2_data[i]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }
        break;
    }

    case START_PID_ORI:
    {
        pid_ori_i = 0;
        start_time = (float)millis();
        end_time = (float)millis();
        error_ori = 0;
        prev_error_ori = 0;
        error_sum_ori = 0;
        yaw = 0;

        pid_ori_on = true;

        break;
    }

    case STOP_PID_ORI:
    {
        pid_ori_on = false;
        num_vars_data_collection = pid_ori_i;
        drive(0, 0);
        break;
    }

    case GET_PID_DATA_ORI:
    {
        for (int i = 0; i < num_data_msgs; i++)
        {
            tx_estring_value.clear();
            tx_estring_value.append(pid_times[i]);
            tx_estring_value.append(",");

            tx_estring_value.append(pid_current_angles[i]);
            tx_estring_value.append(",");

            tx_estring_value.append(pwm_data[i]);
            tx_estring_value.append(",");

            tx_estring_value.append(pid_err[i]);

            tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }
        break;
    }

    default:
        Serial.print("Invalid Command Type: ");
        Serial.println(cmd_type);
        break;
    }
}

void setup()
{
    pinMode(1, OUTPUT);
    pinMode(2, OUTPUT);
    pinMode(16, OUTPUT);
    pinMode(15, OUTPUT);

    Serial.begin(115200);

    BLE.begin();

    // Set advertised local name and service
    BLE.setDeviceName("Artemis BLE");
    BLE.setLocalName("Artemis BLE");
    BLE.setAdvertisedService(testService);

    // Add BLE characteristics
    testService.addCharacteristic(tx_characteristic_float);
    testService.addCharacteristic(tx_characteristic_string);
    testService.addCharacteristic(rx_characteristic_string);

    // Add BLE service
    BLE.addService(testService);

    // Initial values for characteristics
    // Set initial values to prevent errors when reading for the first time on central devices
    tx_characteristic_float.writeValue(0.0);

    Wire.begin();
    Wire.setClock(400000);
    bool initialized = false;
    while (!initialized)
    {
        myICM.begin(Wire, AD0_VAL);
        Serial.print(F("Initialization of the sensor returned: "));
        Serial.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok)
        {
            Serial.println("Trying again..");
            delay(500);
        }
        else
        {
            initialized = true;
        }
    }

    ////DMP initialization////

    bool DMP_success = true;
    // Initialize the DMP
    DMP_success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
    // Enable the DMP Game Rotation Vector sensor
    DMP_success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
    // Set the DMP output data rate (ODR): value = (DMP running rate / ODR ) - 1
    // E.g. for a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    DMP_success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 4) == ICM_20948_Stat_Ok);
    // Enable the FIFO queue
    DMP_success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
    // Enable the DMP
    DMP_success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
    // Reset DMP
    DMP_success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
    // Reset FIFO
    DMP_success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);
    // Check success
    if (!DMP_success)
    {
        Serial.println("Enabling DMP failed!");
        while (1)
        {
            // Freeze
        }
    }
    ////DMP initialization////

    // Output MAC Address
    Serial.print("Advertising BLE with MAC: ");
    Serial.println(BLE.address());

    BLE.advertise();

    // blink upon start up
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
}

void write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval)
    {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000)
        {
            tx_float_value = 0;
        }

        previousMillis = currentMillis;
    }
}

void read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written())
    {
        handle_command();
    }
}

int pid(float dist, float targetDist)
{
    float currDist = dist;
    float error = dist - targetDist;

    float p_term = Kp * error;

    float dt = 98;
    error_sum += error * dt;

    float i_term = Ki * error_sum;

    float pwm = p_term + i_term;

    if(i_term > 200){
        i_term = 200;
    }
    else if(i_term < -200) {
        i_term = -200;
    }

    if (pwm > 0)
    {
        if (pwm > max_speed)
            pwm = max_speed;

        return pwm;
    }
    else if (pwm < 0)
    {
        if (pwm < -max_speed)
            pwm = -max_speed;

        return pwm;
    }

    return pwm;
}

int pid_ori(float target_angle)
{
    current_time = millis();
    float pid_dt = current_time - prev_time;
    prev_time = current_time;

    icm_20948_DMP_data_t data;
    myICM.readDMPdataFromFIFO(&data);

    // Is valid data available?
    if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
    {
        // We have asked for GRV data so we should receive Quat6
        if ((data.header & DMP_header_bitmap_Quat6) > 0)
        {
            double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
            double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0;
            double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0;

            current_angle = quaternion_to_yaw(q1, q2, q3);
        }
    }

    error_ori = current_angle - target_angle;
    double err_d = (error_ori - prev_error_ori) / pid_dt;
    double err_i = err_i + (error_ori * pid_dt);

    // Calculate speed control signal
    pid_speed = (int)(Kp_ori * error_ori + Ki_ori * err_i + Kd_ori * err_d);
    if (pid_speed > max_speed)
    {
        pid_speed = max_speed;
    }
    if (pid_speed < (-1 * max_speed))
    {
        pid_speed = -1 * max_speed;
    }
    if (pid_speed > 0 && pid_speed < 100)
    {
        pid_speed = 100;
    }
    if (pid_speed > -1 * 100 && pid_speed < 0)
    {
        pid_speed = -1 * 100;
    }
    if (pid_speed > 0)
    {
        drive(-2, pid_speed);
    }
    else if (pid_speed < 0)
    {
        right(2, -1 * pid_speed);
    }
    prev_error_ori = error_ori;
    return pid_speed;
}

////Function to convert quaternions to yaw angle///
double quaternion_to_yaw(double q1, double q2, double q3)
{
    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
    double qw = q0;
    double qx = q2;
    double qy = q1;
    double qz = -q3;
    // yaw (z-axis rotation)
    double t3 = +2.0 * (qw * qz + qx * qy);
    double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw = atan2(t3, t4) * 180.0 / PI;
    return yaw;
}

void drive(int direction, int pwm)
{
    // forwards: 1, back: -1, stop: 0, left: -2, right: 2
    if (direction == 1) // forwards
    {
        analogWrite(1, 0);
        analogWrite(2, pwm);
        analogWrite(16, 0);
        analogWrite(15, pwm * calibration_factor);
    }
    else if (direction == -1) // backwards
    {
        analogWrite(1, pwm);
        analogWrite(2, 0);
        analogWrite(16, pwm);
        analogWrite(15, 0);
    }
    else if (direction == 2) // right
    {
        analogWrite(1, pwm);
        analogWrite(2, 0);
        analogWrite(16, 0);
        analogWrite(15, pwm * calibration_factor);
    }
    else if (direction == -2) // left
    {
        analogWrite(1, 0);
        analogWrite(2, pwm);
        analogWrite(16, pwm * calibration_factor);
        analogWrite(15, 0);
    }
    else // stop
    {
        analogWrite(1, 0);
        analogWrite(2, 0);
        analogWrite(16, 0);
        analogWrite(15, 0);
    }
}

void loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central)
    {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected())
        {
            // Send data
            write_data();
            // Read data
            read_data();

            if (pid_on)
            {
                int distance1;
                int pwm;

                if (distanceSensor1.checkForDataReady())
                {
                    distance1 = distanceSensor1.getDistance();
                    distanceSensor1.clearInterrupt();
                    distanceSensor1.stopRanging();
                    distanceSensor1.startRanging();

                    pwm = pid(distance1, target);
                    // Serial.print("PWM: ");
                    // Serial.println(pwm);

                    if (pid_i < num_data_msgs)
                    {
                        distance1_data[pid_i] = distance1;
                        // Serial.print("distance: ");
                        // Serial.println(distance1_data[pid_i]);
                        pwm_data[pid_i] = pwm;
                        // Serial.print("pwm: ");
                        // Serial.println(pwm_data[pid_i]);
                        times[pid_i] = (float)millis();
                        // Serial.print("time: ");
                        // Serial.println(times[pid_i]);
                        pid_i++;
                    }
                }
                if (pwm > 0)
                {
                    drive(1, pwm);
                }
                else if (pwm < 0)
                {
                    drive(-1, pwm);
                }
                else
                {
                    drive(0, 0);
                }
            }

            if (pid_ori_on)
            {
                int pwm;
                pwm = pid_ori(target_angle);

                if (pid_i < num_data_msgs)
                {
                    pid_current_angles[pid_ori_i] = current_angle;
                    pwm_data[pid_ori_i] = pwm;
                    pid_times[pid_ori_i] = (float)millis();
                    pid_err[pid_ori_i] = error_ori;
                    pid_ori_i++;
                }
            }
        }

        Serial.println("Disconnected");
    }
}