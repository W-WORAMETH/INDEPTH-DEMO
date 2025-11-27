#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <EEPROM.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int8_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joy.h>

#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps612.h"

#include <Dynamixel2Arduino.h>

#include <string>
#include <stdlib.h>
#include <cstring>

#include <Adafruit_NeoPixel.h>

#define PIN_WS2812B 2
#define NUM_PIXELS 1

#define pin_MAG1 26
#define pin_MOTOR_EN 27
#define pin_WhiteLight_EN 19
#define pin_IMUAD0 18
// serial define
#define DXL_SERIAL Serial2
#define DEBUG_SERIAL Serial

#define EXECUTE_EVERY_N_MS(MS, X)      \
  do                                   \
  {                                    \
    static volatile int64_t init = -1; \
    if (init == -1)                    \
    {                                  \
      init = uxr_millis();             \
    }                                  \
    if (uxr_millis() - init > MS)      \
    {                                  \
      X;                               \
      init = uxr_millis();             \
    }                                  \
  } while (0)

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      return false;              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }
enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

//=============== Define Function ===============
struct Struct_wheelvector
{
  float goalrad[4];
  float goalmagnitude[4];

  int magnitudeSign[4];
  float closestAngle[4];
};

void subscription_callback_joy(const void *msgin);
void subscription_callback_cmdvel(const void *msgin);
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
double mapf(double val, double in_min, double in_max, double out_min, double out_max);
void commandOmniwheel(float linear_x, float linear_y, float w, float PercentofRotationSpeed);
void commandDifferential(float linear_x, float linear_y, float w);
void commandSwerve(float linear_x, float linear_y, float w);
void commandSwerveNew(float linear_x, float linear_y, float w);
Struct_wheelvector closestAngle(Struct_wheelvector wheelcommand);
float ClosestAngle(float setpoint, float current);
char *create_name(const std::string &str1, const std::string &str2);
void subscription_callback_motorcmd(const void *msgin);
void subscription_callback_motorenablecmd(const void *msgin);
void subscription_callback_boardcmd(const void *msgin);
void subscription_callback_ledcmd(const void *msgin);
void subscription_callback_connectioncheck(const void *msgin);
void subscription_callback_boardparam(const void *msgin);

void connectionCheck();
void initializeIMU();
// =============================================

// Micro Ros Error loop
// void error_loop()
// {
//   while (1)
//   {
//     digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//     delay(100);
//   }
//   return;
// }

double mapf(double val, double in_min, double in_max, double out_min, double out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// ================================================================
// ===             Dynamixel include and set up part            ===
// ================================================================
const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;

const uint8_t DXL_ALL_ID_CNT = 8;
const uint8_t DXL_ALL_ID_LIST[DXL_ALL_ID_CNT] = {11, 21, 31, 41, 12, 22, 32, 42};
// const uint8_t DXL_ALL_ID_LIST[DXL_ALL_ID_CNT] = { 1, 2, 3, 41, 12, 22, 32, 42};
const uint8_t DXL_VELOCITY_ID_CNT = 4;
const uint8_t DXL_VELOCITY_ID_LIST[DXL_VELOCITY_ID_CNT] = {1, 22, 32, 42};

const uint8_t DXL_POSITION_ID_CNT = 4;
const uint8_t DXL_POSITION_ID_LIST[DXL_POSITION_ID_CNT] = {11, 21, 31, 41};
// const uint8_t DXL_POSITION_ID_LIST[DXL_POSITION_ID_CNT] = { 1, 2, 3, 41};

const uint16_t user_pkt_buf_cap = 256;
uint8_t user_pkt_buf[user_pkt_buf_cap];

const uint16_t user_pkt_buf_cap_torque_en = 256;

uint8_t user_pkt_buf_torque_en[user_pkt_buf_cap_torque_en];

const uint16_t SR_START_ADDR = 126; // Starting Data Addr, Can differ Depending on what address to access
const uint16_t SR_ADDR_LEN = 10;    // Data Length (2+4+4), Can differ depending on how many address to access.

const uint16_t SW_VELOCITY_START_ADDR = 104;
const uint16_t SW_VELOCITY_ADDR_LEN = 4;

const uint16_t SW_POSITION_START_ADDR = 116;
const uint16_t SW_POSITION_PROFILE_VEL_ADDR = 112;
const uint16_t SW_POSITION_ADDR_LEN = 4;

const uint16_t SR_TORQUE_EN_START_ADDR = 64;
const uint16_t SR_TORQUE_EN_ADDR_LEN = 1;

typedef struct sr_data
{
  int16_t present_current;
  int32_t present_velocity;
  int32_t present_position;
} __attribute__((packed)) sr_data_t;

typedef struct sr_data_torque_en
{
  int8_t torque_en;
} __attribute__((packed)) sr_data_torque_en_t;

typedef struct sw_velocity_data
{
  // int32_t goal_position;
  int32_t goal_velocity = 2000;
} __attribute__((packed)) sw_velocity_data_t;

typedef struct sw_position_data
{
  int32_t goal_position;
  // int32_t goal_velocity;
  int32_t goal_velocity_profile;
} __attribute__((packed)) sw_position_data_t;

sr_data_t sr_data[DXL_ALL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ALL_ID_CNT];

sr_data_torque_en_t sr_data_torque_en[DXL_ALL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_torque_en_infos;
DYNAMIXEL::XELInfoSyncRead_t info_torque_en_xels_sr[DXL_ALL_ID_CNT];

sw_velocity_data_t sw_velocity_data[DXL_VELOCITY_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_velocity_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_velocity_xels_sw[DXL_VELOCITY_ID_CNT];

sw_position_data_t sw_position_data[DXL_POSITION_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_position_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_position_xels_sw[DXL_POSITION_ID_CNT];

const int DXL_DIR_PIN = 5; // DYNAMIXEL DIR PIN using auto direction rs485 so set = -1

const float DXL_PROTOCOL_VERSION = 2.0;
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// This namespace is required to use Control table item names
using namespace ControlTableItem;

#define LED_PIN 2

char *create_name(const std::string &str1, const std::string &str2)
{
  // Combine the two strings into a single string
  std::string combined = str1 + str2;

  // Allocate memory for the combined string
  char *name = new char[combined.length() + 1];

  // Copy the combined string into the allocated memory
  std::strcpy(name, combined.c_str());

  return name;
}

void fillSyncReadStructures()
{
  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;

  for (int i = 0; i < DXL_ALL_ID_CNT; i++)
  {
    info_xels_sr[i].id = DXL_ALL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t *)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;

  sr_torque_en_infos.packet.p_buf = user_pkt_buf_torque_en;            //
  sr_torque_en_infos.packet.buf_capacity = user_pkt_buf_cap_torque_en; //
  sr_torque_en_infos.packet.is_completed = false;                      //
  sr_torque_en_infos.addr = SR_TORQUE_EN_START_ADDR;                   //
  sr_torque_en_infos.addr_length = SR_TORQUE_EN_ADDR_LEN;              //
  sr_torque_en_infos.p_xels = info_torque_en_xels_sr;                  //
  sr_torque_en_infos.xel_count = 0;                                    //

  for (int i = 0; i < DXL_ALL_ID_CNT; i++)
  {
    info_torque_en_xels_sr[i].id = DXL_ALL_ID_LIST[i]; //
    info_torque_en_xels_sr[i].p_recv_buf = (uint8_t *)&sr_data_torque_en[i];
    sr_torque_en_infos.xel_count++;
  }
  sr_torque_en_infos.is_info_changed = true;
}

void fillSyncWriteStructures()
{

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_velocity_infos.packet.p_buf = nullptr;
  sw_velocity_infos.packet.is_completed = false;
  sw_velocity_infos.addr = SW_VELOCITY_START_ADDR;
  sw_velocity_infos.addr_length = SW_VELOCITY_ADDR_LEN;
  sw_velocity_infos.p_xels = info_velocity_xels_sw;
  sw_velocity_infos.xel_count = 0;

  for (int i = 0; i < 4; i++)
  {
    info_velocity_xels_sw[i].id = DXL_VELOCITY_ID_LIST[i];
    info_velocity_xels_sw[i].p_data = (uint8_t *)&sw_velocity_data[i].goal_velocity;
    sw_velocity_infos.xel_count++;
  }

  sw_position_infos.packet.p_buf = nullptr;
  sw_position_infos.packet.is_completed = false;
  sw_position_infos.addr = SW_POSITION_START_ADDR;
  sw_position_infos.addr_length = SW_POSITION_ADDR_LEN;
  sw_position_infos.p_xels = info_position_xels_sw;
  sw_position_infos.xel_count = 0;

  for (int i = 0; i < 4; i++)
  {
    info_position_xels_sw[i].id = DXL_POSITION_ID_LIST[i];
    info_position_xels_sw[i].p_data = (uint8_t *)&sw_position_data[i].goal_position;
    sw_position_infos.xel_count++;
  }

  sw_velocity_infos.is_info_changed = true;
  sw_position_infos.is_info_changed = true;
}

class WatchdogTimer
{
public:
  WatchdogTimer(int32_t interval) : watchdog_time(0), timestep(interval) {}

  bool checktimeout()
  {
    watchdog_time++;
    if (watchdog_time > timestep)
    {
      timeout = true;
    }
    return timeout;
  }
  void restart_counter()
  {
    watchdog_time = 0;
    timeout = false;
  }

  int32_t watchdog_time;
  int32_t timestep;
  bool timeout = false;
};
