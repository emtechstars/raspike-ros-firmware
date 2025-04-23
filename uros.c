#include <kernel.h>
#include <micro_ros_asp.h>
#include <t_syslog.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/header.h>

#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <sensor_msgs/msg/range.h>

/* spike hub&pup lib */
#include "pbio/color.h"
#include "spike/hub/button.h"
#include "spike/hub/speaker.h"
#include "spike/pup/colorsensor.h"
#include "spike/pup/motor.h"
#include "spike/pup/ultrasonicsensor.h"
#include <pbdrv/battery.h>
#include <spike/hub/battery.h>
#include <spike/hub/display.h>
#include <spike/hub/imu.h>
#include <spike/hub/light.h>

#include "uros.h"
#include <stdio.h>
#include <time.h>

// u_ros custom message
#include <raspike_uros_msg/msg/button_status_message.h>      //button status
#include <raspike_uros_msg/msg/motor_reset_message.h>        //reset count
#include <raspike_uros_msg/msg/motor_speed_message.h>        //motor speed
#include <raspike_uros_msg/msg/speaker_message.h>            //speaker
#include <raspike_uros_msg/msg/spike_dev_status_message.h>   //device status
#include <raspike_uros_msg/msg/spike_power_status_message.h> //power status
#include <std_msgs/msg/bool.h>                               //bool
#include <std_msgs/msg/int8.h>                               //int8

#define RCCHECK(fn)                                                            \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
      error_loop(temp_rc);                                                     \
    }                                                                          \
  }
#define RCSOFTCHECK(fn)                                                        \
  {                                                                            \
    rcl_ret_t temp_rc = fn;                                                    \
    if ((temp_rc != RCL_RET_OK)) {                                             \
    }                                                                          \
  }

rcl_publisher_t dev_status_publisher;
rcl_publisher_t button_status_publisher;
rcl_publisher_t power_status_publisher;

raspike_uros_msg__msg__SpikeDevStatusMessage device_status;
raspike_uros_msg__msg__ButtonStatusMessage hub_button_msg;

int button_state;
int touch_sensor_state;
int pre_button_state;
int pre_touch_sensor_state;

#include "stm32f4xx_hal_conf.h"
extern volatile HRTCNT hrtcnt_current;
uint32_t get_time_usec(void) {
  uint32_t hrtcnt, hrtcnt_delta, systick_val, elapsed;
  const uint32_t load = SysTick->LOAD + 1;
  do {
    hrtcnt = hrtcnt_current;
    systick_val = SysTick->VAL;
    hrtcnt_delta = hrtcnt_current;
  } while (hrtcnt != hrtcnt_delta);
  elapsed = load - systick_val;
  return hrtcnt + (elapsed * 1000) / load;
}

void error_loop(rcl_ret_t temp_rc) {
  syslog(LOG_NOTICE, "error_loop %d\n", temp_rc);

  hub_display_image(img_sad);         //ディスプレイ表示
  hub_light_on_color(PBIO_COLOR_RED); //ステータスライトを赤く点灯する

  while (1) {
    dly_tsk(100);
  }
}

static inline hub_button_t hub_buttons_pressed(hub_button_t button_candidates) {
  hub_button_t pressed;
  hub_button_is_pressed(&pressed);
  return pressed & button_candidates;
}

static int wait_for_hub_buttons(hub_button_t button_candidates) {
  hub_button_t pressed_button;
  int button_command = 0;

  pressed_button = hub_buttons_pressed(button_candidates);

  if (pressed_button & HUB_BUTTON_BT)
    return 2048;
  if (pressed_button & HUB_BUTTON_LEFT)
    button_command += 1;
  if (pressed_button & HUB_BUTTON_RIGHT)
    button_command += 2;
  if (pressed_button & HUB_BUTTON_CENTER)
    button_command += 16;

  return button_command;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  float hub_angular_velocity[3];

  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    /*imu*/
    hub_imu_get_angular_velocity(hub_angular_velocity);
    for (int i = 0; i < 3; ++i) {
      device_status.angular_velocity[i] = hub_angular_velocity[i];
    }
    hub_imu_get_acceleration(hub_angular_velocity);
    for (int i = 0; i < 3; ++i) {
      device_status.linear_acceleration[i] = hub_angular_velocity[i];
    }

    RCSOFTCHECK(rcl_publish(&dev_status_publisher, &device_status, NULL));

    /*button_status_publisher*/
    button_state = wait_for_hub_buttons(HUB_BUTTON_RIGHT | HUB_BUTTON_LEFT |
                                        HUB_BUTTON_CENTER);
    touch_sensor_state = wait_for_hub_buttons(HUB_BUTTON_BT);
    if (pre_button_state != button_state) {
      hub_button_msg.button = button_state;
      RCCHECK(rcl_publish(&button_status_publisher,
                          (const void *)&hub_button_msg, NULL));
    }
    pre_button_state = button_state;
    if (pre_touch_sensor_state != touch_sensor_state) {
      hub_button_msg.touch_sensor = touch_sensor_state;
      RCCHECK(rcl_publish(&button_status_publisher,
                          (const void *)&hub_button_msg, NULL));
    }
    pre_touch_sensor_state = touch_sensor_state;
  }
}

void uros_task(intptr_t exinf) {
  syslog(LOG_NOTICE, "miro-ROS main task : start");

  // Set transports
  set_microros_transports(UROS_PORTID);

  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  rcl_node_t node;
  RCCHECK(rclc_node_init_default(&node, "spike_state_node", "", &support));

  // Create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &button_status_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(raspike_uros_msg, msg, ButtonStatusMessage),
      "spike_button_status"));
  RCCHECK(rclc_publisher_init_best_effort(
      &dev_status_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(raspike_uros_msg, msg, SpikeDevStatusMessage),
      "spike_device_status"));
  RCCHECK(rclc_publisher_init_best_effort(
      &power_status_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(raspike_uros_msg, msg,
                                  SpikePowerStatusMessage),
      "spike_power_status"));

  // Create timer,
  rcl_timer_t timer;
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10),
                                  timer_callback));

  // Create executor
  rclc_executor_t executor;
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  syslog(LOG_NOTICE, "miro-ROS main task : init done.");

  hub_imu_init();
  hub_speaker_set_volume(100);

  syslog(LOG_NOTICE, "SPIKE init done.");

  hub_display_off();
  hub_display_orientation(PBIO_SIDE_TOP);
  hub_display_image(text_ET); //ディスプレイ表示
  hub_light_off();

  while (1) {
    rclc_executor_spin(&executor);
  }

  // Free resources
  RCCHECK(rcl_publisher_fini(&button_status_publisher, &node));
  RCCHECK(rcl_publisher_fini(&dev_status_publisher, &node));
  RCCHECK(rcl_publisher_fini(&power_status_publisher, &node));
  RCCHECK(rcl_node_fini(&node));
}
