
#include "BLECStringCharacteristic.h"
#include "EString.h"
#include<math.h>
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include "ICM_20948.h"  // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "SparkFun_VL53L1X.h"

#define SERIAL_PORT Serial
#define AD0_VAL   1     // The value of the last bit of the I2C address, on the SparkFun 9DoF IMU breakout the default is 1
#define blinkPin LED_BUILTIN

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "2bb357d9-bff9-49ae-92fe-7935d8da4d69"

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

ICM_20948_I2C myICM;  // Otherwise create an ICM_20948_I2C object

// collect pitch and roll data to plot in python
const int array_size = 1000;
float acc_pitch_raw[array_size];
float acc_pitch_lpf[array_size];
float acc_roll_raw[array_size];
float acc_roll_lpf[array_size];

float gyro_pitch_raw[array_size];
float gyro_roll_raw[array_size];
float gyro_yaw_raw[array_size];

float comp_pitch[array_size];
float comp_roll[array_size];
float comp_yaw[array_size];

float distance1[array_size];
float distance2[array_size];

int times[array_size];

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;
// int times[array_size];
float temps[array_size];

#define XSHUT 6
#define ADDRESS 0x30

SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2(Wire, XSHUT);


//////////// Global Variables ////////////

enum CommandTypes
{
    GET_PITCH_DATA,
    GET_ROLL_DATA,
    GET_YAW_DATA,
    GET_IMU_DATA,
    GET_TOF_DATA,
};

void
handle_command()
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
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        
        case GET_PITCH_DATA:
        {
          float pitch_g = 0, dt = 0;
          unsigned long last_time = millis();
          // collect pitch data
          for(int i = 0; i < array_size; i++)  {
            if (myICM.dataReady()) {
              myICM.getAGMT();
              // accelerometer data
              float pitch_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI;
              acc_pitch_raw[i] = pitch_a;

              // gyro data
              dt = (millis()-last_time)/1000.;
              last_time = millis();
              pitch_g = pitch_g + myICM.gyrY()*dt;
              gyro_pitch_raw[i] = pitch_g;

              // time
              times[i] = (int)millis();
            }
          }

          // dt = 1/(sampling rate), RC = 1/(2*pi*cutoff freq), and alpha=dt/(dt+RC)
          const float alpha = 0.0735;

          acc_pitch_lpf[0] = acc_pitch_raw[0];
          comp_pitch[0] = (1-alpha)*gyro_pitch_raw[0] + alpha*acc_pitch_raw[0];

          for(int n = 1; n < array_size; n++){
            float pitch_raw_curr = acc_pitch_raw[n];
            acc_pitch_lpf[n] = alpha * pitch_raw_curr + (1 - alpha) * acc_pitch_lpf[n - 1];
            acc_pitch_lpf[n - 1] = acc_pitch_lpf[n];

            comp_pitch[n] = (1-alpha)*gyro_pitch_raw[n] + alpha*acc_pitch_lpf[n];

          }

          // send pitch data from the pitch array to python
          for(int i = 0; i < array_size; i++) {
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
          for(int i = 0; i < array_size; i++)  {
            if (myICM.dataReady()) {
              myICM.getAGMT();
              // acc data
              float roll_a = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
              acc_roll_raw[i] = roll_a;

              // gyro data
              dt = (millis()-last_time)/1000.;
              last_time = millis();
              roll_g = roll_g + myICM.gyrX()*dt;
              gyro_roll_raw[i] = roll_g;
              
              // time
              times[i] = (int)millis();
            }
          }

          // dt = 1/(sampling rate), RC = 1/(2*pi*cutoff freq), and alpha=dt/(dt+RC)
          const float alpha = 0.0735;

          acc_roll_lpf[0] = acc_roll_raw[0];
          comp_roll[0] = (1-alpha)*gyro_roll_raw[0] + alpha*acc_roll_raw[0];

          for(int n = 1; n < array_size; n++){
            float roll_raw_curr = acc_roll_raw[n];
            acc_roll_lpf[n] = alpha * roll_raw_curr + (1 - alpha) * acc_roll_lpf[n - 1];
            acc_roll_lpf[n - 1] = acc_roll_lpf[n];

            comp_roll[n] = (1-alpha)*gyro_roll_raw[n] + alpha*acc_roll_lpf[n];

          }

          // send roll data from the roll array to python
          for(int i = 0; i < array_size; i++) {
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
          for(int i = 0; i < array_size; i++)  {
            if (myICM.dataReady()) {
              myICM.getAGMT();
              // gyro data
              dt = (millis()-last_time)/1000.;
              last_time = millis();
              yaw_g = yaw_g + myICM.gyrZ()*dt;
              gyro_yaw_raw[i] = yaw_g;
              comp_yaw[i] = (1-alpha)*gyro_yaw_raw[i];
              
              // time
              times[i] = (int)millis();
            }
          }

          // send roll data from the roll array to python
          for(int i = 0; i < array_size; i++) {
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
          for(int i = 0; i < array_size; i++)  {
            if (myICM.dataReady()) {
              myICM.getAGMT();
              // acc roll data
              float roll_a = atan2(myICM.accX(),myICM.accZ())*180/M_PI;
              acc_roll_raw[i] = roll_a;

              // gyro roll data
              dt = (millis()-last_time)/1000.;
              last_time = millis();
              roll_g = roll_g + myICM.gyrX()*dt;
              gyro_roll_raw[i] = roll_g;

              // accelerometer data
              float pitch_a = atan2(myICM.accY(),myICM.accZ())*180/M_PI;
              acc_pitch_raw[i] = pitch_a;

              // gyro data
              dt = (millis()-last_time)/1000.;
              last_time = millis();
              pitch_g = pitch_g + myICM.gyrY()*dt;
              gyro_pitch_raw[i] = pitch_g;
              
              // gyro data
              dt = (millis()-last_time)/1000.;
              last_time = millis();
              yaw_g = yaw_g + myICM.gyrZ()*dt;
              gyro_yaw_raw[i] = yaw_g;
              comp_yaw[i] = (1-alpha)*gyro_yaw_raw[i];
              
              // time
              times[i] = (int)millis();
            }
          }

          // low pass filters

          acc_pitch_lpf[0] = acc_pitch_raw[0];
          comp_pitch[0] = (1-alpha)*gyro_pitch_raw[0] + alpha*acc_pitch_raw[0];
          acc_roll_lpf[0] = acc_roll_raw[0];
          comp_roll[0] = (1-alpha)*gyro_roll_raw[0] + alpha*acc_roll_raw[0];

          for(int n = 1; n < array_size; n++){
            // lpf pitch
            float pitch_raw_curr = acc_pitch_raw[n];
            acc_pitch_lpf[n] = alpha * pitch_raw_curr + (1 - alpha) * acc_pitch_lpf[n - 1];
            acc_pitch_lpf[n - 1] = acc_pitch_lpf[n];

            comp_pitch[n] = (1-alpha)*gyro_pitch_raw[n] + alpha*acc_pitch_lpf[n];

            // lpf roll
            float roll_raw_curr = acc_roll_raw[n];
            acc_roll_lpf[n] = alpha * roll_raw_curr + (1 - alpha) * acc_roll_lpf[n - 1];
            acc_roll_lpf[n - 1] = acc_roll_lpf[n];

            comp_roll[n] = (1-alpha)*gyro_roll_raw[n] + alpha*acc_roll_lpf[n];

          }

          // send roll data from the roll array to python
          for(int i = 0; i < array_size; i++) {
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

          if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
          {
            Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
            while (1);
          }

          digitalWrite(XSHUT, HIGH);
          Serial.print("Distance Sensor 2 Address: 0x");
          Serial.println(distanceSensor2.getI2CAddress(), HEX);

          if (distanceSensor2.begin() != 0) //Begin returns 0 on a good init
          {
            Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
            while (1);
          }
          Serial.println("Sensors 1 and 2 online!");
          
          distanceSensor1.setDistanceModeShort();
          distanceSensor2.setDistanceModeShort();

          distanceSensor1.startRanging();
          distanceSensor2.startRanging();

          int st = millis();

          for(int i = 0; i < array_size; i++)
          {

            while (!distanceSensor1.checkForDataReady())
            {
              delay(1);
            }
            distance1[i] = distanceSensor1.getDistance();
            // Serial.print("Distance 1: ");
            // Serial.print(distance1[i]);

            while (!distanceSensor2.checkForDataReady())
            {
              delay(1);
            }
            distance2[i] = distanceSensor2.getDistance();
            // Serial.print(" | Distance 2: ");
            // Serial.println(distance2[i]);

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

          // for(int i = 0; i < array_size; i++)
          // {
          //   continue;
          // }

          // et = millis();

          // Serial.print("Time for 500 iterations of empty loop: ");
          // Serial.println(et-st);

          for (int i = 0; i < array_size; i++)
          {
            tx_estring_value.clear();
            tx_estring_value.append(times[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(distance1[i]);
            tx_estring_value.append(",");
            tx_estring_value.append(distance2[i]);
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

void
setup()
{
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
    while( !initialized )
    {
      myICM.begin( Wire, AD0_VAL );
      Serial.print( F("Initialization of the sensor returned: ") );
      Serial.println( myICM.statusString() );
      if( myICM.status != ICM_20948_Stat_Ok ){
        Serial.println( "Trying again.." );
        delay(500);
      }else{
        initialized = true;
      }
    }

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

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}

void
loop()
{
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            // Send data
            write_data();

            // Read data
            read_data();
        }

        Serial.println("Disconnected");
    }
}
