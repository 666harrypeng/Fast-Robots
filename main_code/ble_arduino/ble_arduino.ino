#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>

#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <math.h>
#include <cmath> // basically, the same as <math.h>
#include "BasicLinearAlgebra.h"
// using namespace BLA;               //This allows you to declare a matrix

//////////// BLE UUIDs ////////////
#define BLE_UUID_TEST_SERVICE "93d93744-e9c0-4d25-a198-0af9f06f72a4"

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

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

//////////////////////////////////// Global Variables ////////////////////////////////////
//////// Before Lab4 global variables
// char* timeStamps[50];
// int timeStamps_new_status = 0; // denote whether "timeStamps" has dynamic memory
// float* tempStamps;
// int tempStamps_new_status = 0;  // denote whether "tempStamps" has dynamic memory

//////// Before Lab4 global variables
// SEND_DEBUG_READINGS global variables
#define MAX_DEBUG_MSG_SIZE 1000
int timeData[MAX_DEBUG_MSG_SIZE];
float tof1Data[MAX_DEBUG_MSG_SIZE];
float tof2Data[MAX_DEBUG_MSG_SIZE];
float tof1RawData[MAX_DEBUG_MSG_SIZE];
float yawData[MAX_DEBUG_MSG_SIZE];
float leftPWM[MAX_DEBUG_MSG_SIZE];
float rightPWM[MAX_DEBUG_MSG_SIZE];
float distErrorData[MAX_DEBUG_MSG_SIZE]; // PID forward control error Data collection array
float p_data[MAX_DEBUG_MSG_SIZE];
float i_data[MAX_DEBUG_MSG_SIZE];
float d_data[MAX_DEBUG_MSG_SIZE];
float yawErrorData[MAX_DEBUG_MSG_SIZE];
float powerData[MAX_DEBUG_MSG_SIZE];
float target_yaw_data[MAX_DEBUG_MSG_SIZE];
int drift_status_data[MAX_DEBUG_MSG_SIZE];
int motor_direction[MAX_DEBUG_MSG_SIZE];

int info_index = 0;
// PID control initialization
float dt;
int power;
float kp = 0.20;
float ki = 0.05;
float kd = 0.3;
float dist_error;
float prev_error; // for PID fd control
float sum_error;
float cur_dist;
int tof_read_num;
float prev_tof_dist;
float cur_tof_dist;
int prev_tof_time;
int cur_tof_time;
float P;
float I;
float D;
float set_kp;
float set_ki;
float set_kd;
unsigned long prev_time = 0;
int time_gap;
float slope;
float pred_dist;
float target_dist = 330.0;
float tof1_distance;
float tof2_distance;
float trans;
int forward_clamp = 150;
int set_forward_clamp;
float set_fd_cali_factor;
float cali_factor = 0.75;
float set_trans;
float safe_dist = 310; //unit: mm -> just more than one tile length

float kp_orient = 4;
float ki_orient = 0.8;
float kd_orient = 50;
int rotation_direction = 0; // 0->turn left; 1->turn right
float prev_orient_error = 0.0;
float set_orient_kp;
float set_orient_ki;
float set_orient_kd;
int orient_clamp = 215; //210 is good 
float cur_yaw;
float prev_yaw;
float yaw_error;
float target_yaw = 180.0;
float prev_yaw_error;
float sum_yaw_error;
float avg_yaw_error;
float set_yaw_error;

// Gryoscope initialization
float gyr_pitch = 0.0;
float gyr_roll = 0.0;
float gyr_yaw = 0.0;
float pitch_time = 0.0;
float roll_time = 0.0;
float yaw_time = 0.0;
// apply complementary filter
float alpha = 0.2;
float comple_pitch = gyr_pitch;
float comple_roll = gyr_roll;
float comple_yaw = gyr_yaw;
float pre_time = millis();
unsigned long rotate_start_time = 0;
// ToF Reading ini
int tof_reading_time[MAX_DEBUG_MSG_SIZE];

// Kalman Filter
float drag = 0.8057573730364931;
float momentum = 1.3580776120750313;
float sigma_1 = 0.05; // trust in modeled position
float sigma_2 = 0.1;  // trust in modeled velocity
float sigma_3 = 0.2;  // trust in measurement
BLA::Matrix<1, 1> Sigma_z = {sigma_3 * sigma_3};
BLA::Matrix<2, 2> Sigma_u = {sigma_1 * sigma_1, 0,
                             0, sigma_2 *sigma_2};

BLA::Matrix<2, 2> A = {0, 1,
                       0, -drag / momentum};
BLA::Matrix<2, 1> B = {0, 1 / momentum};
BLA::Matrix<1, 2> C = {-1, 0};
BLA::Matrix<2, 2> I2 = {1, 0,
                        0, 1};
BLA::Matrix<2, 1> mu_p;
BLA::Matrix<2, 1> mu = {0, 0}; // state (x)
BLA::Matrix<2, 2> sigma = {sigma_3, 0,
                           0, sigma_3};

// Drift Stunt
int drift_status = 0;

// Mapping Related
int angle_increment = 20;
int angle_collection_number = 360 / angle_increment; // each increment at 20 degrees
float robot_angles[18];
float tof1_mapping_readings[18][10];
float tof2_mapping_readings[18][10];


////////////////////////////////////////////////////////////////////////////////////////

#define SHUTDOWN_PIN 8 // for ToF
#define INTERRUPT_PIN 3

SFEVL53L1X distanceSensor1; // front
SFEVL53L1X distanceSensor2; // side

////////////////////////////////////////////////////////////////////////////////////////
#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 0

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif
////////////////////////////////////////

enum CommandTypes
{
  ECHO,
  TOF_READINGS,
  GO_FORWARD,
  PID_STATUS_INI,
  PID_FD,
  GYR_READINGS,
  SEND_PID_FD_DEBUG_READINGS,
  PID_ORIENT,
  SEND_PID_ORIENT_DEBUG_READINGS,
  KALMAN_FILTER_TEST,
  DRIFT_STUNT,
  SEND_DRIFT_DEBUG_READINGS,
  PID_ANGLE_INCREMENT,
  SEND_PID_ANGLE_INCREMENT_DEBUG_READINGS,
  SET_PID_VALUES,
  SET_ORIENT_PID_VALUES,
  PID_ROTATE,
  PID_TRANS,
  SET_FORWARD_SPEED,
  SET_FD_CALI,
};

void handle_command()
{
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(), rx_characteristic_string.valueLength());

  bool success;
  int cmd_type = -1;

  success = robot_cmd.get_command_type(cmd_type);

  // Check if the last tokenization was successful and return if failed
  if (!success)
  {
    return;
  }

  // Handle the command type accordingly
  switch (cmd_type)
  {
  case ECHO:
  {
    char char_arr[MAX_MSG_SIZE];
    // Extract the next value from the command string as a character array
    success = robot_cmd.get_next_value(char_arr);

    if (!success)
      return;

    // Append "Robot says -> " & ":)"
    char aug_char_arr[MAX_MSG_SIZE];
    int char_arr_len;
    char_arr_len = strlen(char_arr);
    char front_char_arr[MAX_MSG_SIZE];
    strcpy(front_char_arr, "Robot says -> ");
    int front_char_len;
    front_char_len = strlen(front_char_arr);

    if (char_arr_len >= (MAX_MSG_SIZE - (front_char_len + 2)))
    { // no enough space to append
      break;
    }
    else
    {                                        // able to append
      strcpy(aug_char_arr, front_char_arr);  // update the front chars
      for (int i = 0; i < char_arr_len; i++) // update the middle core chars
      {
        aug_char_arr[front_char_len + i] = char_arr[i];
      }
      aug_char_arr[front_char_len + char_arr_len] = ':';
      aug_char_arr[front_char_len + char_arr_len + 1] = ')';
      aug_char_arr[front_char_len + char_arr_len + 2] = '\0';
    }

    // update tx_estring_value and transmit
    tx_estring_value.clear();
    tx_estring_value.append(aug_char_arr);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    Serial.print("Sent back: ");
    Serial.println(tx_estring_value.c_str());

    break;
  }

  case TOF_READINGS:
  {
    int dist_reading_num = 0;
    // sensors activation
    distanceSensor1.startRanging(); // Write configuration bytes to initiate measurement
    distanceSensor2.startRanging(); // Write configuration bytes to initiate measurement
    Serial.print("started ranging");
    Serial.println();
    // while (true)
    while (dist_reading_num <= 2000)
    {
      while (!distanceSensor1.checkForDataReady() || !distanceSensor2.checkForDataReady())
      {
        delay(1);
      }

      // if (distanceSensor1.checkForDataReady() && distanceSensor2.checkForDataReady())
      // {
      // "dist_reading_num" is initialized as the global variable -> int dist_reading_num = 0;
      unsigned long tof_start_time;
      tof_start_time = millis();
      // Serial.print("T: ");
      // Serial.print(tof_start_time);
      // Serial.print('\t');

      int distance1 = distanceSensor1.getDistance(); // Get the result of the measurement from the sensor
      // Serial.print("Distance1 (mm): ");
      // Serial.print(distance1);
      // Serial.print('\t');

      int distance2 = distanceSensor2.getDistance(); // Get the result of the measurement from the sensor
      // Serial.print("Distance2 (mm): ");
      // Serial.print(distance2);

      // Serial.println();

      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(int(tof_start_time));
      tx_estring_value.append("|Dist1:");
      tx_estring_value.append(distance1);
      tx_estring_value.append("|Dist2:");
      tx_estring_value.append(distance2);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());

      Serial.print("Sent back: ");
      Serial.println(tx_estring_value.c_str());
      dist_reading_num++;
      // }
    }
    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();
    distanceSensor2.clearInterrupt();
    distanceSensor2.stopRanging();
    break;
  }

  case GO_FORWARD:
  {
    delay(3000);
    // forward
    // calibration set:
    // (100, 50, 0.5)->almost move
    // (150, 106.5, 0.71),
    // (200, 161, 0.805)
    unsigned long prev_time = millis();
    while (millis() - prev_time < 2000)
    {
      float cali_factor = 0.805;
      int fd_general_power = 200;
      analogWrite(A16, 0);
      analogWrite(A15, fd_general_power * cali_factor); // right
      analogWrite(4, fd_general_power);
      analogWrite(A5, 0); // left
      // delay(1000);
    }

    // stop
    analogWrite(A16, 255);
    analogWrite(A15, 255);
    analogWrite(4, 255);
    analogWrite(A5, 255);
    delay(500);

    break;
  }

  case PID_STATUS_INI:
  {
    int PID_status = 0;
    success = robot_cmd.get_next_value(PID_status);
    if (!success)
      return;

    if (PID_status == 1)
    {
      for (int i = 0; i < MAX_MSG_SIZE; i++)
      {
        timeData[i] = 0;
        tof1Data[i] = 0.0;
        tof2Data[i] = 0.0;
        tof1RawData[i] = 0.0;
        yawData[i] = 0.0;
        leftPWM[i] = 0.0;
        rightPWM[i] = 0.0;
        distErrorData[i] = 0.0;
        p_data[i] = 0.0;
        i_data[i] = 0.0;
        d_data[i] = 0.0;
        yawErrorData[i] = 0.0;
        powerData[i] = 0.0;
        target_yaw_data[i] = 0.0;
        drift_status_data[i] = 0;
        motor_direction[i] = 0;
      }
      power = 0;
      dist_error = 0.0;
      prev_error = 0.0;
      sum_error = 0.0;
      cur_dist = getSensor1_Distance();
      tof_read_num = 0;
      prev_tof_dist = 0.0;
      cur_tof_dist = 0.0;
      prev_tof_time = 0;
      cur_tof_time = 0;
      tof1_distance = 0.0;
      tof2_distance = 0.0;
      trans = 0.0;

      P = 0.0;
      I = 0.0;
      D = 0.0;
      time_gap = 0;
      slope = 0.0;
      pred_dist = 0.0;
      info_index = 0;
      cur_yaw = 0.0;
      prev_yaw = 0.0;
      yaw_error = 0.0;
      prev_yaw_error = 0.0;
      sum_yaw_error = 0.0;
      drift_status = 0;
      avg_yaw_error = 0.0;

      rotation_direction = 0;

      prev_orient_error = 0.0;

      // mapping related
      for (int i = 0; i < angle_collection_number; i++)
      {
        
        robot_angles[i] = 0.0;
        for (int j = 0; j < 10; j++)
        {
          tof2_mapping_readings[i][j] = 0.0;
          tof1_mapping_readings[i][j] = 0.0;
        }
      }

      Serial.println("PID_Status ON");
    }
    else
    {
      Serial.println("PID_Status OFF");
    }
    break;
  }

  case PID_FD:
  {
    // kp = 0.20;
    // ki = 0;
    // kd = 0;

    distanceSensor1.startRanging();
    dist_error = cur_dist - target_dist;

    prev_time = millis();

    delay(80); // avoid ToF sending wired data at the beginning
    // while (dist_error != 0)
    while (abs(dist_error) > 5 && millis() - prev_time <= 10000)
    {

      if (info_index < MAX_DEBUG_MSG_SIZE) // add time stamp
      {
        timeData[info_index] = int(millis() - prev_time);
      }

      cur_dist = getSensor1_Distance();

      // ///// extrapolation ///////
      // if (distanceSensor1.checkForDataReady() == 1 && cur_dist != cur_tof_dist) // tof has updated
      // {
      //   tof_read_num++;
      //   tof_reading_time[tof_read_num - 1] = timeData[info_index];
      //   prev_tof_dist = cur_tof_dist;
      //   cur_tof_dist = cur_dist;
      //   // distanceSensor1.clearInterrupt();
      // }
      // else if (tof_read_num >= 2) // tof has not updated yet -> do extrapolation
      // {
      //   time_gap = tof_reading_time[tof_read_num - 1] - tof_reading_time[tof_read_num - 2];
      //   slope = static_cast<float>(cur_tof_dist - prev_tof_dist) / time_gap;
      //   pred_dist = tof1Data[info_index - 1] + slope * (timeData[info_index] - timeData[info_index - 1]);
      //   cur_dist = pred_dist;
      // }
      // //////// extrapolation done ////////

      ///////////// Kalman Filter //////////////////////////
      if (distanceSensor1.checkForDataReady() == 1 && cur_dist != cur_tof_dist) // tof has updated
      {
        tof_read_num++;
        tof_reading_time[tof_read_num - 1] = timeData[info_index];
        prev_tof_dist = cur_tof_dist;
        cur_tof_dist = cur_dist;
      }

      if (tof_read_num >= 2) // tof has not updated yet -> KF
      {
        kf(mu, sigma, power, -cur_dist, Sigma_u, Sigma_z);
        pred_dist = mu(0, 0);
        cur_dist = pred_dist;
      }
      ///////////// KF finish ///////////////////////////////////////

      dist_error = cur_dist - target_dist;

      PID_position_control(dist_error, kp, ki, kd);

      if (info_index < MAX_DEBUG_MSG_SIZE)
      {
        tof1RawData[info_index] = cur_tof_dist;
        tof1Data[info_index] = cur_dist;
        distErrorData[info_index] = dist_error;
        info_index++;
      }
    }

    // stop
    analogWrite(A16, 0);
    analogWrite(A15, 0);
    analogWrite(4, 0);
    analogWrite(A5, 0);
    delay(200);

    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();

    break;
  }

  case GYR_READINGS:
  {
    while (true)
    {
      // if (myICM.dataReady())
      // {
      //   myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
      //   // printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
      //   // printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
      SERIAL_PORT.print("Yaw:");
      // dmp_get_yaw(&myICM);
      cur_yaw = get_gyr_yaw(&myICM);
      SERIAL_PORT.println(cur_yaw);
      //     // SERIAL_PORT.println(get_gyr_yaw(&myICM));
      //     delay(30);
      //   }
      //   else
      //   {
      //     SERIAL_PORT.println("Waiting for data");
      //     delay(500);
      //   }
    }
    break;
  }
  case SEND_PID_FD_DEBUG_READINGS:
  {
    for (int i = 0; i < MAX_DEBUG_MSG_SIZE; i++)
    {
      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(timeData[i]);
      tx_estring_value.append("|Dist1:");
      tx_estring_value.append(tof1Data[i]);
      tx_estring_value.append("|Dist2:");
      tx_estring_value.append(tof2Data[i]);
      tx_estring_value.append("|leftPWM:");
      tx_estring_value.append(leftPWM[i]);
      tx_estring_value.append("|rightPWM:");
      tx_estring_value.append(rightPWM[i]);
      tx_estring_value.append("|errorData:");
      tx_estring_value.append(distErrorData[i]);
      tx_estring_value.append("|pData:");
      tx_estring_value.append(p_data[i]);
      tx_estring_value.append("|iData:");
      tx_estring_value.append(i_data[i]);
      tx_estring_value.append("|dData:");
      tx_estring_value.append(d_data[i]);
      tx_estring_value.append("|Dist1Raw:");
      tx_estring_value.append(tof1RawData[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }

  case PID_ORIENT:
  {
    orient_clamp = 230;

    prev_time = millis();

    delay(80);
    myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
    // dmp_get_yaw(&myICM);
    cur_yaw = get_gyr_yaw(&myICM);
    target_yaw = cur_yaw + 45.0;

    yaw_error = target_yaw - cur_yaw;

    while (millis() - prev_time <= 8000)
    {
      if (info_index < MAX_DEBUG_MSG_SIZE)
      {
        timeData[info_index] = int(millis() - prev_time);
      }

      myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
      // dmp_get_yaw(&myICM);
      cur_yaw = get_gyr_yaw(&myICM);      
      yaw_error = target_yaw - cur_yaw;

      if (info_index >= 1)
      {
        if (abs(yaw_error) >= 2)
        {
          PID_orient_control(yaw_error, kp_orient, ki_orient, kd_orient);
        }
        else // stop
        {
          // re-initialize the integration term
          sum_yaw_error = 0.0;
          prev_yaw_error = 0.0;

          motor_hard_stop();
          delay(200);
        }
      }

      if (info_index < MAX_DEBUG_MSG_SIZE)
      {
        yawData[info_index] = cur_yaw;
        yawErrorData[info_index] = yaw_error;
        info_index++;
      }
    }

    // stop
    motor_hard_stop();
    delay(200);

    motor_soft_stop();
    delay(10);
    break;
  }

  case SEND_PID_ORIENT_DEBUG_READINGS:
  {
    for (int i = 0; i < info_index; i++)
    {
      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(timeData[i]);
      tx_estring_value.append("|Yaw:");
      tx_estring_value.append(yawData[i]);
      tx_estring_value.append("|leftPWM:");
      tx_estring_value.append(leftPWM[i]);
      tx_estring_value.append("|rightPWM:");
      tx_estring_value.append(rightPWM[i]);
      tx_estring_value.append("|YawError:");
      tx_estring_value.append(yawErrorData[i]);
      tx_estring_value.append("|pData:");
      tx_estring_value.append(p_data[i]);
      tx_estring_value.append("|iData:");
      tx_estring_value.append(i_data[i]);
      tx_estring_value.append("|dData:");
      tx_estring_value.append(d_data[i]);
      tx_estring_value.append("|Dir:");
      tx_estring_value.append(motor_direction[i]);
      tx_estring_value.append("|power:");
      tx_estring_value.append(powerData[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }

  case KALMAN_FILTER_TEST:
  {
    distanceSensor1.startRanging();
    cur_tof_dist = getSensor1_Distance();
    dist_error = cur_tof_dist - target_dist;
    unsigned long start_time = millis();
    unsigned long cur_time = 0;
    int pwm_value = 0;
    while (dist_error >= 0)
    {
      // Serial.println("in while loop");

      cur_tof_dist = getSensor1_Distance();

      dist_error = cur_tof_dist - target_dist;

      cur_time = millis() - start_time;

      if (cur_time <= 2000) // delay 2000ms
      {
        // Serial.println("in delay");

        delay(1);
        if (info_index < MAX_DEBUG_MSG_SIZE)
        {
          timeData[info_index] = cur_time;
          tof1Data[info_index] = cur_tof_dist;
          prev_tof_dist = cur_tof_dist;

          info_index++;
        }
      }
      else
      {
        // Serial.println("running");
        pwm_value = 80;
        analogWrite(A16, 0);
        analogWrite(A15, 45); // right
        analogWrite(4, 90);
        analogWrite(A5, 0); // left
        if (info_index < MAX_DEBUG_MSG_SIZE && cur_tof_dist != prev_tof_dist)
        {
          timeData[info_index] = cur_time;
          tof1Data[info_index] = cur_tof_dist;
          prev_tof_dist = cur_tof_dist;
          leftPWM[info_index] = pwm_value;
          info_index++;
        }
      }
    }
    // stop
    // Serial.println("stop");

    analogWrite(A16, 0);
    analogWrite(A15, 0);
    analogWrite(4, 0);
    analogWrite(A5, 0);
    delay(200);

    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();

    break;
  }

  case DRIFT_STUNT:
  {
    distanceSensor1.startRanging();
    dist_error = cur_dist - target_dist;

    prev_time = millis();

    delay(80); // avoid ToF sending wired data at the beginning

    gyr_yaw = 0.0;
    myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
    dmp_get_yaw(&myICM);
    // target_yaw = cur_yaw;

    drift_status = 0;

    while (millis() - prev_time <= 8000)
    {
      cur_dist = getSensor1_Distance();

      ///////////// Kalman Filter //////////////////////////
      if (drift_status == 1) // after drift
      {
        tof_read_num = 0;
        sum_yaw_error = 0.0;  // pid orient control initialization
        prev_yaw_error = 0.0; // pid orient control initialization
        prev_error = 0.0;     // pid position control initialization
        sum_error = 0.0;      // pid position control initialization
      }
      if (distanceSensor1.checkForDataReady() == 1 && cur_dist != cur_tof_dist) // tof has updated
      {
        tof_read_num++;
        tof_reading_time[tof_read_num - 1] = timeData[info_index];
        prev_tof_dist = cur_tof_dist;
        cur_tof_dist = cur_dist;
      }

      if (tof_read_num >= 2) // tof has not updated yet -> KF
      {
        kf(mu, sigma, power, -cur_dist, Sigma_u, Sigma_z);
        pred_dist = mu(0, 0);
        cur_dist = pred_dist;
      }
      ///////////// KF finish ///////////////////////////////////////

      dist_error = cur_dist - target_dist;

      myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
      // cur_yaw = get_gyr_yaw(&myICM);
      dmp_get_yaw(&myICM);

      if (dist_error <= 1300 && drift_status == 0)
      {
        target_yaw = cur_yaw + 180.0;
        drift_status = 1;
        kp = 4.5;
        ki = 0.05;
        kd = 2;
        forward_clamp = 130;
      }

      yaw_error = target_yaw - cur_yaw;

      if (drift_status == 0)
      {
        rotate_start_time = millis();
      }

      while (drift_status == 1 && abs(yaw_error) > 3 && millis() - rotate_start_time <= 3000)
      {
        PID_orient_control(yaw_error, kp_orient, ki_orient, kd_orient);
        myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
        // cur_yaw = get_gyr_yaw(&myICM);
        dmp_get_yaw(&myICM);
        yaw_error = target_yaw - cur_yaw;

        if (info_index < MAX_DEBUG_MSG_SIZE)
        {
          timeData[info_index] = int(millis() - prev_time);
          yawData[info_index] = cur_yaw;
          target_yaw_data[info_index] = target_yaw;
          yawErrorData[info_index] = yaw_error;
          drift_status_data[info_index] = drift_status;
          tof1Data[info_index] = cur_dist;
          distErrorData[info_index] = dist_error;
          tof1RawData[info_index] = cur_tof_dist;
          // p_data[info_index] = P;
          // i_data[info_index] = I;
          // d_data[info_index] = D;
          // powerData[info_index] = power;
          info_index++;
        }

        if (abs(yaw_error) < 3)
        {
          break;
        }
      }

      PID_position_control(dist_error, kp, ki, kd);

      if (info_index < MAX_DEBUG_MSG_SIZE)
      {
        timeData[info_index] = int(millis() - prev_time);
        yawData[info_index] = cur_yaw;
        target_yaw_data[info_index] = target_yaw;
        yawErrorData[info_index] = yaw_error;
        drift_status_data[info_index] = drift_status;
        tof1Data[info_index] = cur_dist;
        distErrorData[info_index] = dist_error;
        tof1RawData[info_index] = cur_tof_dist;
        // p_data[info_index] = P;
        // i_data[info_index] = I;
        // d_data[info_index] = D;
        // powerData[info_index] = power;
        info_index++;
      }
    }

    // stop
    analogWrite(A16, 0);
    analogWrite(A15, 0);
    analogWrite(4, 0);
    analogWrite(A5, 0);
    delay(200);

    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();
    break;
  }

  case SEND_DRIFT_DEBUG_READINGS:
  {
    Serial.println("Start to send back drift debug readings");

    for (int i = 0; i < MAX_DEBUG_MSG_SIZE; i++)
    {
      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(timeData[i]);
      tx_estring_value.append("|Yaw:");
      tx_estring_value.append(yawData[i]);
      tx_estring_value.append("|tarYaw:");
      tx_estring_value.append(target_yaw_data[i]);
      tx_estring_value.append("|YawErr:");
      tx_estring_value.append(yawErrorData[i]);
      tx_estring_value.append("|driSta:");
      tx_estring_value.append(drift_status_data[i]);
      tx_estring_value.append("|D1:");
      tx_estring_value.append(tof1Data[i]);
      tx_estring_value.append("|DErr:");
      tx_estring_value.append(distErrorData[i]);
      tx_estring_value.append("|D1Raw:");
      tx_estring_value.append(tof1RawData[i]);
      tx_estring_value.append("|p:");
      tx_estring_value.append(p_data[i]);
      tx_estring_value.append("|i:");
      tx_estring_value.append(i_data[i]);
      tx_estring_value.append("|d:");
      tx_estring_value.append(d_data[i]);
      tx_estring_value.append("|PWM:");
      tx_estring_value.append(powerData[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }

  case PID_ANGLE_INCREMENT:
  {
    distanceSensor1.startRanging();
    distanceSensor2.startRanging();


    orient_clamp = 230; // rotation speed (duty cycle)

    delay(100);
    prev_time = millis();

    myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
    cur_yaw = get_gyr_yaw(&myICM);

    int incre_counts = 0; // for robot_angles array index count


    // each increment at 20 degrees -> 18 datapoints in total
    for (int i = 0; i < angle_collection_number; i++)
    {
      gyr_yaw = 0.0;
      cur_yaw = 0.0;
      myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
      cur_yaw = get_gyr_yaw(&myICM);

      yaw_error = angle_increment;
      // target_yaw = angle_increment + cur_yaw;
      target_yaw = angle_increment + cur_yaw;
      while (abs(yaw_error) > 0.8)
      {
        if (info_index < MAX_DEBUG_MSG_SIZE)
        {
          timeData[info_index] = int(millis() - prev_time);
        }

        myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
        cur_yaw = get_gyr_yaw(&myICM);
        yaw_error = target_yaw - cur_yaw;

        Serial.print(" yaw err = ");
        Serial.print(yaw_error);

        if (info_index >= 1)
        {
          PID_orient_control(yaw_error, kp_orient, ki_orient, kd_orient);
        }

        myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
        cur_yaw = get_gyr_yaw(&myICM);
        yaw_error = target_yaw - cur_yaw;

        // record the debug readings data

        Serial.print(" cur_yaw = ");
        Serial.print(cur_yaw);
        Serial.println();

        Serial.print(" p = ");
        Serial.print(P);
        Serial.print(" I = ");
        Serial.print(I);
        Serial.print(" D = ");
        Serial.print(D);
        Serial.println();

        Serial.print("Rotation direction = ");
        Serial.print(rotation_direction);
        Serial.println();

        if (info_index < MAX_DEBUG_MSG_SIZE)
        {
          yawData[info_index] = cur_yaw;
          yawErrorData[info_index] = yaw_error;
          info_index++;
        }
      }

      // record the distances
      for (int j = 0; j < 10; j++)
      {
        distanceSensor1.startRanging();
        distanceSensor2.startRanging();
        tof1_distance = getSensor1_Distance();
        tof2_distance = getSensor2_Distance();
        distanceSensor1.clearInterrupt();
        distanceSensor1.stopRanging();
        distanceSensor2.clearInterrupt();
        distanceSensor2.stopRanging();
        tof1_mapping_readings[incre_counts][j] = tof1_distance;
        tof2_mapping_readings[incre_counts][j] = tof2_distance;
      }

      // re-initialize the yaw's reading
      sum_yaw_error = 0.0;
      prev_yaw_error = 0.0;
      

      // record the finalized angle
      // robot_angles[incre_counts] = cur_yaw;
      robot_angles[incre_counts] = cur_yaw + angle_increment * incre_counts;
      Serial.print("    robot_angle = ");
      Serial.print(robot_angles[incre_counts]);
      Serial.println();
      incre_counts++;

      gyr_yaw = 0.0;
      cur_yaw = 0.0;

      // stop
      motor_hard_stop();
      delay(800);
    }

    // stop
    motor_soft_stop();
    delay(500); 
    break;
  }

  case SEND_PID_ANGLE_INCREMENT_DEBUG_READINGS:
  {
    int data_kind; // 0->continuous PID control results  1->finalized angles for each increment
    success = robot_cmd.get_next_value(data_kind);
    if (!success)
    {
      return;
    }

    if (data_kind == 0) // send continuous PID control results
    {
      for (int i = 0; i < MAX_DEBUG_MSG_SIZE; i++)
      {
        tx_estring_value.clear();
        tx_estring_value.append("T:");
        tx_estring_value.append(timeData[i]);
        tx_estring_value.append("|Yaw:");
        tx_estring_value.append(yawData[i]);
        tx_estring_value.append("|YawErr:");
        tx_estring_value.append(yawErrorData[i]);
        tx_characteristic_string.writeValue(tx_estring_value.c_str());
      }
    }
    else if (data_kind == 1) // send finalized angles for each increment
    {
      for (int i = 0; i < angle_collection_number; i++)
      {
        for (int j = 0; j < 10; j++)
        {
          tx_estring_value.clear();
          tx_estring_value.append("Yaw:");
          tx_estring_value.append(robot_angles[i]);
          tx_estring_value.append("|Tof1:");
          tx_estring_value.append(tof1_mapping_readings[i][j]);
          tx_estring_value.append("|Tof2:");
          tx_estring_value.append(tof2_mapping_readings[i][j]);
          tx_characteristic_string.writeValue(tx_estring_value.c_str());
        }
      }
    }

    break;
  }

  case SET_PID_VALUES:
  {
    success = robot_cmd.get_next_value(set_kp);
    if (!success)
      return;
    success = robot_cmd.get_next_value(set_ki);
    if (!success)
      return;
    success = robot_cmd.get_next_value(set_kd);
    if (!success)
      return;
    kp = set_kp;
    ki = set_ki;
    kd = set_kd;

    Serial.print(" P:");
    Serial.print(kp);
    Serial.print(" I:");
    Serial.print(ki);
    Serial.print(" D:");
    Serial.print(kd);
    Serial.println();
    break;
  }

  case SET_ORIENT_PID_VALUES:
  {
    success = robot_cmd.get_next_value(set_orient_kp);
    if (!success)
      return;
    success = robot_cmd.get_next_value(set_orient_ki);
    if (!success)
      return;
    success = robot_cmd.get_next_value(set_orient_kd);
    if (!success)
      return;
    kp_orient = set_orient_kp;
    ki_orient = set_orient_ki;
    kd_orient = set_orient_kd;

    Serial.print(" kp_orient:");
    Serial.print(kp_orient);
    Serial.print(" ki_orient:");
    Serial.print(ki_orient);
    Serial.print(" kd_orient:");
    Serial.print(kd_orient);
    Serial.println();
    break;
  }
  
  case PID_ROTATE:  // for the path planning PID rotate
  {
    success = robot_cmd.get_next_value(set_yaw_error);
    if (!success)
    {
      return;
    }

    yaw_error = set_yaw_error;

    Serial.print("yaw_error = ");
    Serial.println(yaw_error);
    
    orient_clamp = 230;

    gyr_yaw = 0.0;
    cur_yaw = 0.0;

    prev_time = millis();
    myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
    cur_yaw = get_gyr_yaw(&myICM);

    target_yaw = cur_yaw + yaw_error;

    Serial.print("Start PID Rotate");
    Serial.println();

    unsigned long prev_rotate_time;
    prev_rotate_time = millis();

    while (abs(yaw_error) > 1)
    {
      if (info_index < MAX_DEBUG_MSG_SIZE)
      {
        timeData[info_index] = int(millis() - prev_rotate_time);
      }
      myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
      cur_yaw = get_gyr_yaw(&myICM);      
      
      yaw_error = target_yaw - cur_yaw;

      Serial.print("yaw_error = ");
      Serial.println(yaw_error);

      PID_orient_control(yaw_error, kp_orient, ki_orient, kd_orient);

      info_index++;
      // Serial.print("Info_index = ");
      // Serial.print(info_index);
      // Serial.println();

    }

    // stop
    motor_hard_stop();
    delay(50);

    motor_soft_stop();

    Serial.print("Finish PID Rotate");
    Serial.println();
    break;
  }

  case PID_TRANS: // for the path planning PID forward moving
  {
    success = robot_cmd.get_next_value(set_trans);
    if (!success)
    {
      return;
    }

    // unit change from ft -> mm
    trans = set_trans * 304.8;  
    Serial.print("Trans = ");
    Serial.println(trans);

    distanceSensor1.startRanging();
    cur_dist = getSensor1_Distance();
    target_dist = cur_dist - trans;
    dist_error = trans; // namely -> dist_error = cur_dist - target_dist;
    
    Serial.print("Start PID Trans");
    Serial.println();


    orient_clamp = 230;

    gyr_yaw = 0.0;
    cur_yaw = 0.0;

    myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
    cur_yaw = get_gyr_yaw(&myICM);

    target_yaw = cur_yaw;

    unsigned long prev_fd_time;
    prev_fd_time = millis();


    while (abs(dist_error) > 2 && cur_dist > 200)
    { 
      Serial.print("PID Loop!");
      Serial.println();

      cur_dist = getSensor1_Distance();
      ///////////// Kalman Filter //////////////////////////
      if (distanceSensor1.checkForDataReady() == 1 && cur_dist != cur_tof_dist) // tof has updated
      {
        tof_read_num++;
        tof_reading_time[tof_read_num - 1] = timeData[info_index];
        prev_tof_dist = cur_tof_dist;
        cur_tof_dist = cur_dist;
      }

      if (tof_read_num >= 2) // tof has not updated yet -> KF
      {
        kf(mu, sigma, power, -cur_dist, Sigma_u, Sigma_z);
        pred_dist = mu(0, 0);
        cur_dist = pred_dist;
      }
      ///////////// KF finish ///////////////////////////////////////


      // check distance safety
      if ((cur_dist - safe_dist) < 100) 
      {
        // target_dist = safe_dist;
        motor_hard_stop();
        delay(1000);
        Serial.print("Safety distance!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        Serial.println();
        break;
      }

      dist_error = cur_dist - target_dist;

      if (info_index < MAX_DEBUG_MSG_SIZE)
      {
        timeData[info_index] = int(millis() - prev_fd_time);
      }

      myICM.getAGMT(); // The values are only updated when you call 'getAGMT'
      cur_yaw = get_gyr_yaw(&myICM);  

      yaw_error = target_yaw - cur_yaw;

      // PID_fd_with_angle_control(dist_error, kp, ki, kd, yaw_error, kp_orient, ki_orient, kd_orient);
      PID_position_control(dist_error, kp, ki, kd);

      info_index++;

      Serial.print("PID loop end");
      Serial.println();

    }

    // stop
    motor_hard_stop();
    delay(1000);

    motor_soft_stop();

    distanceSensor1.clearInterrupt();
    distanceSensor1.stopRanging();

    Serial.print("Finish PID Trans");
    Serial.println();
    break;
  }

  case SET_FORWARD_SPEED:
  {

    success = robot_cmd.get_next_value(set_forward_clamp);
    if (!success)
    {
      return;
    }

    forward_clamp = set_forward_clamp;

    Serial.print(" fd clamp = ");
    Serial.print(forward_clamp);
    Serial.println();

    break;
  }

  case SET_FD_CALI:
  {
    success = robot_cmd.get_next_value(set_fd_cali_factor);
    if (!success)
    {
      return;
    }

    cali_factor = set_fd_cali_factor;
    
    Serial.print("FD cali factor = ");
    Serial.print(cali_factor);
    Serial.println();

    break;
  }
  default:
  {
    Serial.print("Invalid Command Type: ");
    Serial.println(cmd_type);
    break;
  }
  }
}

void kf(BLA::Matrix<2, 1> &mu, BLA::Matrix<2, 2> &sigma, BLA::Matrix<1, 1> u,
        BLA::Matrix<1, 1> y, BLA::Matrix<2, 2> Sigma_u, BLA::Matrix<1, 1> Sigma_z)
{
  dt = (timeData[info_index] - timeData[info_index - 1]) * 0.001;
  // discretize matrices
  BLA::Matrix<2, 2> Ad; // Ad -> (2, 2)
  Ad(0, 0) = 1 + dt * A(0, 0);
  Ad(0, 1) = 0 + dt * A(0, 1);
  Ad(1, 0) = 0 + dt * A(1, 0);
  Ad(1, 1) = 1 + dt * A(1, 1);
  BLA::Matrix<2, 1> Bd;
  Bd(0, 0) = dt * B(0, 0);
  Bd(1, 0) = dt * B(1, 0);
  // mu -> (2, 1)
  // prediction and update block
  mu_p = Ad * mu + Bd * u;
  BLA::Matrix<2, 2> sigma_p = Ad * sigma * (~Ad) + Sigma_u;
  BLA::Matrix<1, 1> sigma_m = C * sigma_p * (~C) + Sigma_z;
  BLA::Matrix<1, 1> sigma_m_invert;
  sigma_m_invert(0, 0) = 1 / sigma_m(0, 0);
  BLA::Matrix<2, 1> kkf_gain = sigma_p * (~C) * sigma_m_invert;
  BLA::Matrix<1, 1> y_m = y - C * mu_p;
  mu = mu_p + kkf_gain * y_m;
  sigma = (I2 - kkf_gain * C) * sigma_p;
}

void PID_fd_with_angle_control(float dist_error, float kp, float ki, float kd, 
                            float yaw_error, float kp_orient, float ki_orient, float kd_orient)
{
  // calculate the fd PID part
  dt = (timeData[info_index] - timeData[info_index - 1]) * 0.001; // unit: s
  P = kp * dist_error;

  
  if (info_index == 0)
  {
    prev_error = dist_error;
  }

  D = kd * (dist_error - prev_error) / dt;
  prev_error = dist_error;

  sum_error = sum_error + dist_error * dt;
  I = ki * sum_error;
  int I_clamp = 20;

  if (I > I_clamp)
  {
    I = I_clamp;
  }
  else if (I < -I_clamp)
  {
    I = -I_clamp;
  }

  int fd_power = P + I + D;

  int fd_power_clamp = 2000;
  if (fd_power < -fd_power_clamp)
  {
    fd_power = -fd_power_clamp;
  }
  else if (fd_power > fd_power_clamp)
  {
    fd_power = fd_power_clamp;
  }
  //linear reflect to PWM
  int fd_deadband = 100;
  int fd_upper_bound = 255;
  int fd_pwm_value = fd_deadband + (fd_upper_bound - fd_deadband) * (fd_power + fd_power_clamp) / (2 * fd_power_clamp); // Adjusting range from [-fd_power_clamp, fd_power_clamp] to [0, 2*fd_power_clamp]

  // calculate the angle PID part
  float P_orient = kp_orient * yaw_error;
  float D_orient = kd_orient * (yaw_error - prev_yaw_error) / dt; // normal pid
  prev_yaw_error = yaw_error;
  if (prev_yaw_error * yaw_error < 0) // different signs
  {
    sum_yaw_error = 0.0;
  }
  sum_yaw_error = sum_yaw_error + yaw_error * dt;
  float I_orient = ki_orient * sum_yaw_error;
  int I_orient_clamp = 60;
  if (I_orient > I_orient_clamp)
  {
    I_orient = I_orient_clamp;
  }
  else if (I_orient < -I_orient_clamp)
  {
    I_orient = -I_orient_clamp;
  }
  int orient_power = P_orient + I_orient + D_orient;
  int orient_power_clamp = 2000;
  if (orient_power < -orient_power_clamp)
  {
    orient_power = -orient_power_clamp;
  }
  else if (orient_power > orient_power_clamp)
  {
    orient_power = orient_power_clamp;
  }

  int orient_deadband = 120;
  int orient_upper_bound = 255;
  int orient_pwm_value = orient_deadband + (orient_upper_bound - orient_deadband) * (orient_power + orient_power_clamp) / (2 * orient_power_clamp);



  // calculate the overall pid
  int overall_power = fd_pwm_value + orient_pwm_value;
  int overall_power_clamp = 300;
  if (overall_power < -overall_power_clamp)
  {
    overall_power = -overall_power_clamp;
  }
  else if (overall_power > overall_power_clamp)
  {
    overall_power = overall_power_clamp;
  }
  //linear reflect to PWM
  int deadband = 100;
  int pwm_upper_bound = forward_clamp;
  int pwm_value = deadband + (pwm_upper_bound - deadband) * (overall_power + overall_power_clamp) / (2 * overall_power_clamp); // Adjusting range from [-1000, 1000] to [0, 2000]

  Serial.print("fd_power = ");
  Serial.print(fd_power);
  Serial.print("  orient_power = ");
  Serial.print(orient_power);
  Serial.print("  fd_pwm = ");
  Serial.print(fd_pwm_value);
  Serial.print("  orient_pwm = ");
  Serial.print(orient_pwm_value);
  Serial.println();
  Serial.print("      overall pwm= ");
  Serial.print(pwm_value);
  Serial.println();

  if (overall_power < 0)
  { // backward
    analogWrite(A15, 0);
    analogWrite(A16, pwm_value * cali_factor); // right
    analogWrite(A5, pwm_value);
    analogWrite(4, 0); // left
    // delay(5);
  }
  else if (overall_power >= 0)
  { // forward
    analogWrite(A16, 0);
    analogWrite(A15, pwm_value * cali_factor); // right
    analogWrite(4, pwm_value);
    analogWrite(A5, 0); // left
    // delay(10);
  }
  if (info_index < MAX_DEBUG_MSG_SIZE)
  {
    powerData[info_index] = overall_power;
    leftPWM[info_index] = pwm_value;
    rightPWM[info_index] = pwm_value * cali_factor;
    p_data[info_index] = P;
    i_data[info_index] = I;
    d_data[info_index] = D;
  }


}

void PID_orient_control(float yaw_error, float kp_orient, float ki_orient, float kd_orient)
{ 
  
  //turn left -> yaw increase (anti-clockwise)
  //turn right -> yaw decrease
  //yaw_error = target_yaw - cur_yaw
  //error < 0 => target_yaw < cur_yaw => yaw should decrease => turn right
  //error > 0 => target_yaw > cur_yaw => yaw should increase => turn left
  

  float cali_factor = 0.75;
  int deadband = 180;
  dt = (timeData[info_index] - timeData[info_index - 1]) * 0.001; // unit: s
  // Serial.print("  info_index = ");
  // Serial.print(info_index);
  // Serial.print("  dt = ");
  // Serial.print(dt);
  // Serial.println();

  P = kp_orient * yaw_error;

  // if (info_index == 0)
  // {
  //   prev_yaw_error = yaw_error;
  // }


  D = kd_orient * (yaw_error - prev_yaw_error) / dt; // normal pid
  // D = kd_orient * (-yaw_error + prev_yaw_error) / dt;  // "gradient descend"
  prev_yaw_error = yaw_error;

  if (prev_yaw_error * yaw_error < 0) // different signs
  {
    sum_yaw_error = 0.0;
  }

  sum_yaw_error = sum_yaw_error + yaw_error * dt;
  I = ki_orient * sum_yaw_error;


  // Serial.print(" yaw_err = ");
  // Serial.print(yaw_error);
  // Serial.print(" sum_yaw_error = ");
  // Serial.print(sum_yaw_error);
  // Serial.print(" dt = ");
  // Serial.print(dt);
  // Serial.print(" ki = ");
  // Serial.print(ki_orient);
  // Serial.print(" I = ");
  // Serial.print(I);



  int I_clamp = 60;
  if (I > I_clamp)
  {
    I = I_clamp;
  }
  else if (I < -I_clamp)
  {
    I = -I_clamp;
  }

  // Serial.print(" Clamped_I = ");
  // Serial.print(I);
  // Serial.println();

  // int power = P + I;
  int power = P + I + D;
  // int power = P + D;
  // int power = P ;

  // rotation_direction = 0; // 0->turn left; 1->turn right

  if (power < 0)  // turn right
  {
    rotation_direction = 1;

    if (power < -orient_clamp)
    {
      power = -orient_clamp;
    }
    else if (power > -deadband)
    {
      power = -deadband;
    }
    int left_power = abs(power);                     
    int right_power = abs(power * cali_factor); 
    
    // left side fd & right side back
    analogWrite(A16, right_power);
    analogWrite(A15, 0); // right
    analogWrite(A5, 0);
    analogWrite(4, left_power); // left

    // Serial.print("left:");
    // Serial.print(left_power);
    // Serial.print(" right:");
    // Serial.print(right_power);
    // Serial.println();
  }
  else if (power >= 0) // turn left
  {                    

    rotation_direction = 0;

    if (power > orient_clamp)
    {
      power = orient_clamp;
    }
    else if (power < deadband)
    {
      power = deadband;
    }
    int left_power = abs(power);
    int right_power = abs(power * cali_factor); 

    // left side back & right side fd
    analogWrite(A15, right_power);
    analogWrite(A16, 0); // right
    analogWrite(4, 0);
    analogWrite(A5, left_power); // left

    // Serial.print("left:");
    // Serial.print(left_power);
    // Serial.print(" right:");
    // Serial.print(right_power);
    // Serial.println();
  }

    // Serial.println("Power = ");
    // Serial.println(power);
    // Serial.print("P = ");
    // Serial.print(P);
    // Serial.print("  I = ");
    // Serial.print(I);
    // Serial.print("  D = ");
    // Serial.print(D);
    // Serial.println();



  if (info_index < MAX_DEBUG_MSG_SIZE)
  {
    // Serial.print("power:");
    // Serial.print(power);
    // Serial.println();

    powerData[info_index] = power;
    leftPWM[info_index] = power;
    rightPWM[info_index] = power * cali_factor;
    p_data[info_index] = P;
    i_data[info_index] = I;
    d_data[info_index] = D;
    motor_direction[info_index] = rotation_direction;
  }
}

void PID_position_control(float dist_error, float kp, float ki, float kd)
{
  // float cali_factor = 0.75;
  dt = (timeData[info_index] - timeData[info_index - 1]) * 0.001; // unit: s
  P = kp * dist_error;

  if (info_index == 0)
  {
    prev_error = dist_error;
    // P = kp * dist_error + 100;  //65 is deadband
  }

  D = kd * (dist_error - prev_error) / dt;
  prev_error = dist_error;

  sum_error = sum_error + dist_error * dt;
  I = ki * sum_error;
  int I_clamp = 20;
  if (I > I_clamp)
  {
    I = I_clamp;
  }
  else if (I < -I_clamp)
  {
    I = -I_clamp;
  }

  // int power = P + I;
  int power = P + I + D;
  // int power = P + D;
  // int power = P ;

  // power clamp
  int fd_power_clamp = 1000;
  if (power < -fd_power_clamp)
  {
    power = -fd_power_clamp;
  }
  else if (power > fd_power_clamp)
  {
    power = fd_power_clamp;
  }
  //linear reflect to PWM
  int deadband = 100;
  int pwm_value = deadband + (forward_clamp - deadband) * (power + fd_power_clamp) / fd_power_clamp; // Adjusting range from [-1000, 1000] to [0, 2000]


  Serial.print("power = ");
  Serial.print(power);
  Serial.print("  pwm value = ");
  Serial.print(pwm_value);
  Serial.println();

  if (power < 0)
  { // backward
    // int back_power = 0;
    // if (power < -forward_clamp)
    // {
    //   power = -forward_clamp;
    //   back_power = forward_clamp;
    // }
    // else if (power > -deadband)
    // {
    //   power = -deadband;
    //   back_power = deadband;
    // }
    // else
    // {
    //   back_power = -power;
    // }
    analogWrite(A15, 0);
    analogWrite(A16, pwm_value * cali_factor); // right
    analogWrite(A5, pwm_value);
    analogWrite(4, 0); // left
    // delay(5);
  }
  else if (power >= 0)
  { // forward
    // if (power > forward_clamp)
    // {
    //   power = forward_clamp;
    // }
    // else if (power < deadband)
    // {
    //   power = deadband;
    // }
    analogWrite(A16, 0);
    analogWrite(A15, pwm_value * cali_factor); // right
    analogWrite(4, pwm_value);
    analogWrite(A5, 0); // left
    // delay(10);
  }


  if (info_index < MAX_DEBUG_MSG_SIZE)
  {
    powerData[info_index] = power;
    leftPWM[info_index] = power;
    rightPWM[info_index] = power * cali_factor;
    p_data[info_index] = P;
    i_data[info_index] = I;
    d_data[info_index] = D;
  }
}

float getSensor1_Distance()
{
  float distance1;
  distance1 = distanceSensor1.getDistance(); // Get the result of the measurement from the sensor
  return distance1;
}

float getSensor2_Distance()
{
  float distance2;
  distance2 = distanceSensor2.getDistance(); // Get the result of the measurement from the sensor
  return distance2;
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

void printScaledAGMT(ICM_20948_I2C *sensor)
{
  // gyr tasks
  SERIAL_PORT.print("Pitch: ");
  printFormattedFloat(get_gyr_pitch(sensor), 2, 2);

  SERIAL_PORT.print("	Roll: ");
  printFormattedFloat(get_gyr_roll(sensor), 2, 2);

  SERIAL_PORT.print("	Yaw: ");
  printFormattedFloat(get_gyr_yaw(sensor), 2, 2);
  SERIAL_PORT.println();

  // complementary filter
  //   SERIAL_PORT.print("Pitch: ");
  //   printFormattedFloat(comple_filter_pitch(sensor), 2, 2);
  //   SERIAL_PORT.print("	Roll: ");
  //   printFormattedFloat(comple_filter_roll(sensor), 2, 2);
  //   SERIAL_PORT.print("	Yaw: ");
  //   printFormattedFloat(comple_filter_yaw(sensor), 2, 2);
  //   SERIAL_PORT.println();
  delay(10);
}

float get_accel_pitch(ICM_20948_I2C *sensor)
{
  return atan2(sensor->accX(), sensor->accY()) * 180 / M_PI;
}
float get_accel_roll(ICM_20948_I2C *sensor)
{
  return atan2(sensor->accY(), sensor->accZ()) * 180 / M_PI;
}

float get_gyr_pitch(ICM_20948_I2C *sensor)
{
  unsigned long cur_time = millis();
  float dt = float((cur_time - pitch_time) / 1000.0);
  gyr_pitch += (sensor->gyrY() * dt);
  pitch_time = cur_time;
  return gyr_pitch;
}
float get_gyr_roll(ICM_20948_I2C *sensor)
{
  unsigned long cur_time = millis();
  float dt = float((cur_time - roll_time) / 1000.0);
  gyr_roll += (sensor->gyrX() * dt);
  roll_time = cur_time;
  return gyr_roll;
}

float get_gyr_yaw(ICM_20948_I2C *sensor)
{
  unsigned long cur_time = millis();
  float dt = float((cur_time - yaw_time) / 1000.0);
  gyr_yaw += (sensor->gyrZ() * dt);
  yaw_time = cur_time;
  return gyr_yaw;
}

void dmp_get_yaw(ICM_20948_I2C *sensor)
{
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      double yaw = atan2(t3, t4) * 180.0 / PI; // range: [-180, 180)

      // handle the sudden change between 180 and -180
      float deltaYaw = float(yaw) - cur_yaw;
      if (deltaYaw > 180)
      {
        // Wraparound from 180 to -180
        deltaYaw -= 360;
      }
      else if (deltaYaw < -180)
      {
        // Wraparound from -180 to 180
        deltaYaw += 360;
      }
      // Add the corrected change in yaw to the previous yaw
      cur_yaw = cur_yaw + deltaYaw;
    }
  }
  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
}

float comple_filter_pitch(ICM_20948_I2C *sensor)
{
  float cur_time = millis();
  float dt = float((cur_time - pre_time) / 1000.0);
  comple_pitch = (comple_pitch + sensor->gyrY() * dt) * (1 - alpha) + get_accel_pitch(sensor) * alpha;
  pre_time = cur_time;
  return comple_pitch;
}
float comple_filter_roll(ICM_20948_I2C *sensor)
{
  float cur_time = millis();
  float dt = float((cur_time - pre_time) / 1000.0);
  comple_roll = (comple_roll + sensor->gyrX() * dt) * (1 - alpha) + get_accel_roll(sensor) * alpha;
  pre_time = cur_time;
  return comple_roll;
}
float comple_filter_yaw(ICM_20948_I2C *sensor)
{
  float cur_time = millis();
  float dt = float((cur_time - pre_time) / 1000.0);
  comple_yaw = comple_yaw + sensor->gyrY() * dt;
  pre_time = cur_time;
  return comple_yaw;
}

void motor_soft_stop()
{
  analogWrite(A16, 0);
  analogWrite(A15, 0);
  analogWrite(4, 0);
  analogWrite(A5, 0);
}

void motor_hard_stop()
{
  analogWrite(A16, 255);
  analogWrite(A15, 255);
  analogWrite(4, 255);
  analogWrite(A5, 255);
}

void setup()
{
  Serial.begin(115200);
  //////////////////////////////////////  BLE Setup  ////////////////////////////////////////////////////////////////////////////
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

  // Output MAC Address
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();

  //////////////////////////////////////  ToF Setup  ////////////////////////////////////////////////////////////////////////////
  Wire.begin();

  pinMode(SHUTDOWN_PIN, OUTPUT);
  digitalWrite(SHUTDOWN_PIN, LOW);
  distanceSensor2.setI2CAddress(0x30);

  digitalWrite(SHUTDOWN_PIN, HIGH);

  distanceSensor1.begin();
  distanceSensor2.begin();

  Serial.println("VL53L1X Qwiic Test");
  if (distanceSensor1.begin() != 0) // Begin returns 0 on a good init
  {
    Serial.println("Sensor1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  else if (distanceSensor2.begin() != 0)
  {
    Serial.println("Sensor2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }

  Serial.println("Sensors online!");

  // distanceSensor1.setDistanceModeShort(); //1.3m
  distanceSensor1.setDistanceModeLong(); // 4m   actually can detect 4300mm

  // distanceSensor2.setDistanceModeShort(); //1.3m  actually can detect 2000mm
  distanceSensor2.setDistanceModeLong(); // 4m

  distanceSensor1.startRanging();
  distanceSensor2.startRanging();

  while (distanceSensor1.checkForDataReady() != 1)
  {
    // Serial.println("distance sensor 1 not ready ");
    delay(1);
  }
  while (distanceSensor2.checkForDataReady() != 1)
  {
    // Serial.println("distance sensor 2 not ready ");
    delay(1);
  }
  cur_dist = getSensor1_Distance();

  // distanceSensor1.stopRanging();
  // distanceSensor1.clearInterrupt();

  //////////////////////////////////////////  IMU Setup //////////////////////////////////////////////////////////////

  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT)
  {
  };

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  // myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  SERIAL_PORT.println(F("IMU connected!"));


  ICM_20948_fss_t myFSS;
  myFSS.g = dps1000; // (ICM_20948_GYRO_CONFIG_1_FS_SEL_e)
                  // dps250
                  // dps500
                  // dps1000
                  // dps2000

  myICM.setFullScale(ICM_20948_Internal_Gyr, myFSS);

  Serial.println("IMU Sensor DPS initialized");

//   bool success = true; // Use success to show if the DMP configuration was successful

//   // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
//   success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

//   // Enable the DMP Game Rotation Vector sensor
//   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

//   // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
//   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
//                                                                                // Enable the FIFO
//   success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

//   // Enable the DMP
//   success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

//   // Reset DMP
//   success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

//   // Reset FIFO
//   success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

//   // Check success
//   if (success)
//   {
// #ifndef QUAT_ANIMATION
//     SERIAL_PORT.println(F("DMP enabled!"));
// #endif
//   }
//   else
//   {
//     SERIAL_PORT.println(F("Enable DMP failed!"));
//     SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
//     while (1)
//       ; // Do nothing more
//   }

  /////////////////////////////////////////// Motor Setup ////////////////////////////////////////////////////////////
  pinMode(4, OUTPUT);  // xin1
  pinMode(A5, OUTPUT); // xin2  // left motor

  pinMode(A16, OUTPUT); // xin1 // right motor
  pinMode(A15, OUTPUT); // xin2

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // LED visual indication Blink the LED three times
  pinMode(LED_BUILTIN, OUTPUT); // Set the LED pin as output
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
    delay(500);                     // Wait for a second
    digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off
    delay(500);                     // Wait for a second
  }
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
      // write_data();
      // accel_x = myimu.acelX()
      // pitch = atan2(accel_x, accel_z)
      // if start_data_record and i <100:
      //   pitch_data[i] = pitch
      //   time_stamp[i] = millis()
      //   i++
      // Read data
      // while (PID_run_case== 1)
      // {

      // }
      // if (PID_run_case == 0)
      // {
      //   STOP();
      // }
      read_data();
    }

    Serial.println("Disconnected");
  }
}
