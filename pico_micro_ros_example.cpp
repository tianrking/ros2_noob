#include <stdio.h>

#include "hardware/pio.h"
#include "hardware/timer.h"
#include "hardware/pwm.h"
#include "quadrature_encoder.pio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#define RCCHECK(fn)                                                                      \
    {                                                                                    \
        rcl_ret_t temp_rc = fn;                                                          \
        if ((temp_rc != RCL_RET_OK))                                                     \
        {                                                                                \
            printf("Failed status on line %d: %d. Aborting.\n", __LINE__, (int)temp_rc); \
            return 1;                                                                    \
        }                                                                                \
    }
#define RCSOFTCHECK(fn)                                                                    \
    {                                                                                      \
        rcl_ret_t temp_rc = fn;                                                            \
        if ((temp_rc != RCL_RET_OK))                                                       \
        {                                                                                  \
            printf("Failed status on line %d: %d. Continuing.\n", __LINE__, (int)temp_rc); \
        }                                                                                  \
    }

const uint LED_PIN = 25;

rcl_publisher_t publisher_base;
std_msgs__msg__Int32 msg_base;

rcl_publisher_t publisher_diy;
std_msgs__msg__Int32 msg_diy;

rcl_subscription_t subscriber_base;
std_msgs__msg__Int32 msg_base_sub;

rcl_subscription_t subscriber_diy;
std_msgs__msg__Int32 msg_diy_sub;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher_base, &msg_base, NULL);
    msg_base.data++;
}

int k = 1;
void subscription_callback_base(const void * msgin)
{
  // Cast received message to used type
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  // Process message
  printf("Received: %d\n", msg->data);
  if(k>0){
        gpio_set_dir(LED_PIN, 1);
  }
  else{
        gpio_set_dir(LED_PIN, 0);
  }
  k*= -1;
}

void subscription_callback_diy(const void * msgin)
{
  // Cast received message to used type
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  // Process message
  printf("Received: %d\n", msg->data);
  if(k>0){
        gpio_set_dir(LED_PIN, 1);
  }
  else{
        gpio_set_dir(LED_PIN, 0);
  }
  k*= -1;
}


int main()
{
    stdio_init_all();

    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher_base,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher_base");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_publisher_init_default(
        &publisher_diy,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher_diy");

    // create subscriber
    rclc_subscription_init_default(
        &subscriber_base,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "subscriber_base");

    // create subscriber
    rclc_subscription_init_default(
        &subscriber_diy,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "subscriber_diy");

    rclc_executor_init(&executor, &support.context, 5, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    rclc_executor_add_subscription(&executor, &subscriber_base, &msg_base_sub, &subscription_callback_base, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &subscriber_diy, &msg_diy_sub, &subscription_callback_diy, ON_NEW_DATA);

    gpio_put(LED_PIN, 1);

    msg_base.data = 0;

    msg_diy.data = 1;

    int wrap;
    int GPIO_motor_pwm = 6;
    gpio_set_function(GPIO_motor_pwm, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(GPIO_motor_pwm);
    wrap = 62500; // 2khz
    // wrap = 12500 ; //should be 10 khz right
    pwm_set_wrap(slice_num, wrap);
    pwm_set_enabled(slice_num, true);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0.5 * 62500);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        rcl_ret_t ret_diy = rcl_publish(&publisher_diy, &msg_diy, NULL);
        msg_diy.data *= -1;

        // printf("Work Well\n");

        sleep_ms(2);
    }
    return 0;
}
