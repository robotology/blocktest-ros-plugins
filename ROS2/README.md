# 1. blocktest-ros2-plugins

Repository containing the ROS2 plugins for blocktest


# 2. Installation

Supported OS: Linux Ubuntu 20.04.
Boost library version must be >1.64.


## 2.1. Prerequisite Linux

Install the following:
```bash
sudo apt-get install -y cmake libboost-all-dev
```

### 2.1.1. ROS2
**ROS2 Foxy** version see https://docs.ros2.org/en/rolling/Installation.html

For position controlll install:
```bash
apt-get install ros-foxy-ros2-control
apt-get install ros-foxy-ros2-controllers
```

### 2.1.2. Blocktest
See https://github.com/robotology/blocktest

### 2.1.3. Robometry(optional)
In order to compile `actionTopicRobometry`, install [`robometry`](https://github.com/robotology/robometry) following [these instructions](https://github.com/robotology/robometry#installation-from-sources) and then compile with `ENABLE_ROBOMETRY=ON`.


## 2.2. Installation and compilation

To compile just execute the following commands in a bash
terminal.
```bash
git clone https://github.com/robotology/blocktest-ros2-plugins     
cd blocktest-ros2-plugins
mkdir build
cd build
ccmake ..
```
Make sure your CMAKE_INSTALL_PREFIX in ccmake is \<path to your blocktest install dir\>

Suggested location:
```
~\blocktest\install
```

Make sure your blocktestcore_DIR is \<path to your blocktest build dir\>

Suggested location:
```
~\blocktest\install
```

Make sure your blocktestcore_INCLUDE_DIR is \<path to your blocktest src dir\>



Then:

```bash
make -j 4
make install
```

**Note**
You can add to ~/.bashrc
```
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:~/Documents/install
source /opt/ros/foxy/setup.bash
```


# 3. Available commands
These action blocks are contained in blocktest-ros2-plugin.

## 3.1. ROS commands

-   **rostopicread**

    Read from ROS2 topic.
    ROS2 supported types:
    - std_msgs::msg::String  
      ```{"std_msg_String":{"data":"tosend"}}```

    - geometry_msgs::msg::Twist  
      ```{"geometry_msgs_Twist":{"x":1,"y":2,"z":3,"xa":4,"ya":0,"za":0}}```

    - sensor_msgs::msg::JointState
    ```{"sensor_msgs_JointState":{"name":"ankle","position":2,"velocity":3,"effort":4}}```
    
    - std_msgs::msg::Float64MultiArray
      ```{"std_msgs_Float64MultiArray":{"list":[1, 2, 3, 4]}}```
    
  Example:  
  ```xml
        <command name='rostopicread' topic='' expected='{"std_msg_String":{"data":"tosend"}}' receivertimeout="10000" repetitions='1' wait='0' reporterror='true'/>
  ```
Note that the `receivedtimeout` in in millisecoonds.

-   **rostopicwrite**

    Write to ROS2 topic.
    ROS2 supported types:
    - std_msgs::msg::String  
      ```{"std_msg_String":{"data":"tosend"}}```

    - geometry_msgs::msg::Twist  
      ```{"geometry_msgs_Twist":{"x":1,"y":2,"z":3,"xa":4,"ya":0,"za":0}}```
    
    - sensor_msgs::msg::JointState
    ```{"sensor_msgs_JointState":{"name":"ankle","position":2,"velocity":3,"effort":4}}```
    
    - std_msgs::msg::Float64MultiArray
      ```{"std_msgs_Float64MultiArray":{"list":[1, 2, 3, 4]}}```

Example:

```xml
    <command name='rostopicwrite' topic='' data='{"geometry_msgs_Twist":{"x":1,"y":2,"z":3,"xa":4,"ya":0,"za":0}}' repetitions='1' wait='0' reporterror='true'/>
```

-   **rostopicrobometry**

    Read from a ROS2 topic and dump using [`robometry`](https://github.com/robotology/robometry).
    ROS2 supported types:
    
    - sensor_msgs::msg::JointState
    ```{"sensor_msgs_JointState":{"name":"ankle","position":2,"velocity":3,"effort":4}}```

Example:

```xml
    <command library="ros" name="rostopicrobometry" topic="" repetitions="1" wait="0" reporterror="true" receivertimeout="10000"></command>
```


# 4. Troubleshooting

- If tests don't work check ROS2 version and be sure that you have only one version installed.
- If ROS2 doesn't start check if you have executed the `preliminary ROS2 bash script`.
- Be careful with quotes and double quotes in tests.

# 5. Debug tools

## 5.1. Subscribe to topics with cmdline

`ros2 topic  echo /mytopic`  
`ros2 topic pub /mytopic std_msgs/String "data: hello"`  


`ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "  {linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`
