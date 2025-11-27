// swerve
#include "HERO_WheelModule.h"
#define Module_name "MotorDrive" // front = 1 //back = 2

// --- NEW: Define Pump Pins ---
#define PIN_SUCTION 26
#define PIN_SUPPLY 19

rcl_subscription_t MotorCMDsubscriber;
// --- NEW: Pump Subscribers ---
rcl_subscription_t SuctionSubscriber;
rcl_subscription_t SupplySubscriber;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
uint8_t i;

std_msgs__msg__Int32 MotorCMDmsg;
// --- NEW: Pump Messages ---
std_msgs__msg__Int32 SuctionMsg;
std_msgs__msg__Int32 SupplyMsg;

Adafruit_NeoPixel WS2812B(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

// --- NEW: Callback for Suction Pump ---
void subscription_callback_suction(const void *msgin)
{
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  int pwm_input = msg->data;

  // Constrain input to 0-100 safety range
  if (pwm_input < 0) pwm_input = 0;
  if (pwm_input > 100) pwm_input = 100;

  // Map 0-100% to 0-255 PWM duty cycle
  int pwm_output = map(pwm_input, 0, 100, 0, 255);
  
  analogWrite(PIN_SUCTION, pwm_output);
}

// --- NEW: Callback for Supply Pump ---
void subscription_callback_supply(const void *msgin)
{
  const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;
  int pwm_input = msg->data;

  // Constrain input to 0-100 safety range
  if (pwm_input < 0) pwm_input = 0;
  if (pwm_input > 100) pwm_input = 100;

  // Map 0-100% to 0-255 PWM duty cycle
  int pwm_output = map(pwm_input, 0, 100, 0, 255);
  
  analogWrite(PIN_SUPPLY, pwm_output);
}

void subscription_callback_motorcmd(const void *msgin)
{
  const std_msgs__msg__Int32 *MotorCMDmsg = (const std_msgs__msg__Int32 *)msgin;

  sw_velocity_data[0].goal_velocity = MotorCMDmsg->data; 
  if (sw_velocity_data[0].goal_velocity > 0)
  {
    WS2812B.begin();
    WS2812B.setPixelColor(0, WS2812B.Color(0, 200, 0));
    WS2812B.show();
  }
  else if (sw_velocity_data[0].goal_velocity < 0)
  {
    WS2812B.begin();
    WS2812B.setPixelColor(0, WS2812B.Color(200, 0, 0));
    WS2812B.show();
  }
  else
  {
    WS2812B.begin();
    WS2812B.setPixelColor(0, WS2812B.Color(100, 100, 100));
    WS2812B.show();
  }

  sw_velocity_infos.is_info_changed = true;

  if (dxl.syncWrite(&sw_velocity_infos) == true)
  {
    // DEBUG_SERIAL.println(F("[SyncWrite   VELO ] Success"));
  }
  return;
}

bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  char *name = create_name("HEROWheelModuleNode_", String(Module_name).c_str());
  RCCHECK(rclc_node_init_default(&node, name, "", &support));
  delete[] name; 

  // --- Motor Subscriber ---
  name = create_name("INDEPTH/", String(Module_name).c_str());
  RCCHECK(rclc_subscription_init_default(
      &MotorCMDsubscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      name));
  delete[] name;

  // --- NEW: Suction Pump Subscriber ---
  RCCHECK(rclc_subscription_init_default(
      &SuctionSubscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "INDEPTH/SuctionPump"));

  // --- NEW: Supply Pump Subscriber ---
  RCCHECK(rclc_subscription_init_default(
      &SupplySubscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "INDEPTH/SupplyPump"));

  executor = rclc_executor_get_zero_initialized_executor();
  
  // --- UPDATED: Executor size increased to 3 (Motor + Suction + Supply) ---
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

  RCCHECK(rclc_executor_add_subscription(&executor, &MotorCMDsubscriber, &MotorCMDmsg, &subscription_callback_motorcmd, ON_NEW_DATA));
  
  // --- NEW: Add Pump subscriptions to executor ---
  RCCHECK(rclc_executor_add_subscription(&executor, &SuctionSubscriber, &SuctionMsg, &subscription_callback_suction, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &SupplySubscriber, &SupplyMsg, &subscription_callback_supply, ON_NEW_DATA));
  
  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_subscription_fini(&MotorCMDsubscriber, &node);
  // --- NEW: Finish Pump Subscribers ---
  rcl_subscription_fini(&SuctionSubscriber, &node);
  rcl_subscription_fini(&SupplySubscriber, &node);

  rcl_timer_fini(&timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup()
{
  // --- NEW: Set Pump Pins to Output ---
  pinMode(PIN_SUCTION, OUTPUT);
  pinMode(PIN_SUPPLY, OUTPUT);
  
  // Initialize to OFF
  analogWrite(PIN_SUCTION, 0);
  analogWrite(PIN_SUPPLY, 0);

  WS2812B.begin();
  WS2812B.setPixelColor(0, WS2812B.Color(0, 0, 100));
  WS2812B.show();

  DEBUG_SERIAL.begin(115200);

  set_microros_transports();

  //----------------------------------------------------------------------------------------
  //------------------         Dynamixel First Setup                       -----------------
  //----------------------------------------------------------------------------------------
  dxl.begin(1000000);                               // 57600
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION); // Get DYNAMIXEL information

  for (i = 0; i < 4; i++) // set mode position motor
  {
    dxl.torqueOff(DXL_POSITION_ID_LIST[i]);
    dxl.setOperatingMode(DXL_POSITION_ID_LIST[i], OP_POSITION);
    dxl.writeControlTableItem(DRIVE_MODE, DXL_POSITION_ID_LIST[i], 4);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_POSITION_ID_LIST[i], 300);
    dxl.torqueOn(DXL_POSITION_ID_LIST[i]);
  }
  for (i = 0; i < 4; i++) // set mode velocity motor
  {
    dxl.torqueOff(DXL_VELOCITY_ID_LIST[i]);
    dxl.setOperatingMode(DXL_VELOCITY_ID_LIST[i], OP_VELOCITY);
    dxl.torqueOn(DXL_VELOCITY_ID_LIST[i]);
  }
  Serial.println(F("Enable torque"));

  fillSyncReadStructures();
  fillSyncWriteStructures();

  if(create_entities())
  {
      WS2812B.begin();
      WS2812B.setPixelColor(0, WS2812B.Color(100, 100, 100));
      WS2812B.show();
  }
  else
  {
      WS2812B.begin();
      WS2812B.setPixelColor(0, WS2812B.Color(100, 0, 0));
      WS2812B.show();
  }
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
}