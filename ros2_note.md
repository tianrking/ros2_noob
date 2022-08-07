
# Basic

## check ENV

```bash
printenv | grep -i ROS
```

## ros2 status

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
```

## ros2 topic


### Terminal

```bash
ros2 topic pub /{topic_name} {std_msgs/Strings} "data: GG"

ros2 topic echo /{topic_name}
```
### erros

```bash
ros2 msg list // not found
```

## ros2 topic

```bash
ros2 service type /clear
```
## rqt_console

```bash
ros2 run rqt_console rqt_console
```

## Workspace

```bash
mkdir {workspace}
sudo apt install python3-colcon-common-extensions 
colcon build 
colcon build --symlink-install
colcon test
```

- demo

```bash
git clone https://github.com/ros2/examples src/examples -b humble
```

### error

```bash
colcon //command not found
```

## turtlesim

[Offical link](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)

- Install

```bash
sudo apt update
sudo apt install ros-humble-turtlesim
```

- Check package installed

```bash
ros2 pkg executables turtlesim
```

- Start

```bash
ros2 run turtlesim turtlesim_node
```

- Control

```bash
ros2 run turtlesim turtle_teleop_key
```

## rqt

```bash
sudo apt update
sudo apt install ~nros-humble-rqt*
```

```bash
rqt --force-discover
```

```bash
rqt_graph
```

## 

```bash
ros2 run <package_name> <executable_name>
```


# Coding

## Writing a simple publisher and subscriber (C++)


### publisher

rosdep install -i --from-path src --rosdistro humble -y

(base) ➜  cpp_pubsub rosdep install -i --from-path src --rosdistro humble -y
Traceback (most recent call last):
  File "/usr/local/bin/rosdep", line 5, in <module>
    from roscdep2.main import rosdep_main
ModuleNotFoundError: No module named 'roscdep2'

colcon build --packages-select cpp_pubsub

### subscriber 



# micro-ros model

## RCLC API: publisher
rcl_publisher_t ping_publisher;
rclc_publisher_init_default(&ping_publisher, 
 &node, 
 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
 "/microROS/ping");


## RCLC API: subscription
rcl_subscription_t ping_subscriber;
rclc_subscription_init_default(&ping_subscriber, 
 &node, 
 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
 "/microROS/ping");
ping_
publisher
/ping
timer
/pong
ping_
subscriber
pong_
publisher
pong_
subscriber
node
## RCLC API: subscription callback
void ping_subscription_callback(const void * msgin)
{
 const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
 rcl_publish(&pong_publisher, (const void*)msg, NULL);
 }
}

## RCLC API: executor
rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
rclc_executor_init(&executor, &support.context, 3, &allocator));
rclc_executor_add_timer(&executor, &timer));
rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping, 
&ping_subscription_callback, ON_NEW_DATA));
rclc_executor_spin(&executor)

#  Analysise

## Subscription

### Initialization

```bash
// Subscription object
rcl_subscription_t subscriber;
const char * topic_name = "test_topic";

// Get message type support
const rosidl_message_type_support_t * type_support =
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32);

// Initialize best effort subscriber
rcl_ret_t rc = rclc_subscription_init_best_effort(
  &subscriber, &node,
  type_support, topic_name);

if (RCL_RET_OK != rc) {
  ...  // Handle error
  return -1;
}
```

```c++
rcl_node_t node_diy;
rclc_support_t support_diy;
rcl_subscription_t subscriber_diy;

rcl_allocator_t allocator_diy = rcl_get_default_allocator();
rclc_support_init(&support_diy, 0, NULL, &allocator_diy);
rclc_node_init_default(&node_diy, "pico_node", "", &support_diy);

rclc_subscription_init_default(
        &subscriber_diy,
        &node_diy,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "subscriber_diy");

std_msgs__msg__Int32 msg;
rclc_executor_t executor_diy;
rclc_executor_init(&executor_diy, &support_diy.context, 1, &allocator);
rclc_executor_add_subscription(&executor_diy, &subscriber_diy, &msg, &subscription_callback, ON_NEW_DATA)

```

```c++
rcl_subscription_t pong_subscriber;

void pong_subscription_callback(const void * msgin)
{
	const std_msgs__msg__Int32 *msg = (const std_msgs__msg__Int32 *)msgin;

	if (msg->data == 0)
    {
        gpio_put(LED_PIN, 1);
    }
    else
    {
        gpio_put(LED_PIN, 0);
    }
}

int main(){
    ///
RCCHECK(rclc_subscription_init_best_effort(&pong_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong"));
RCCHECK(rclc_executor_add_subscription(&executor, &pong_subscriber, &incoming_pong, &pong_subscription_callback, ON_NEW_DATA));
  
    ///

}
```

### Callbacks
