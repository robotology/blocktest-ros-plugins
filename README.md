# 1. blocktest-ros-plugins


- [1. blocktest-ros-plugins](#1-blocktest-ros-plugins)
- [2. Installation](#2-installation)
  - [2.1. Prerequisite Linux](#21-prerequisite-linux)
  - [2.2. Installation and compilation](#22-installation-and-compilation)
- [3. Available commands](#3-available-commands)
  - [3.1. ROS commands](#31-ros-commands)
- [4. Troubleshooting](#4-troubleshooting)


Repository containing the ROS2 plugins for blocktest


# 2. Installation

Supported OS: Linux Ubuntu 20.04.
Boost library version must be >1.64.


## 2.1. Prerequisite Linux

```bash
sudo apt-get install -y cmake libboost-all-dev
```
**ROS2 Foxy** version see https://docs.ros.org/en/rolling/Installation.html


## 2.2. Installation and compilation

To compile just execute the following commands in a bash
terminal.
```bash
git clone https://github.com/robotology/blocktest-ros-plugins     
cd blocktest-ros-plugins
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

# 3. Available commands
These action blocks are contained in ros blocktest plugin.

## 3.1. ROS commands

-   **rostopicread**

    Read from ROS2 topic

    ROS2 supported types
    - std_msgs::msg::String  
      ```{"std_msg_String":{"data":"tosend"}}```

    - geometry_msgs::msg::Twist  
      ```{"geometry_msgs_Twist":{"x":1,"y":2,"z":3,"xa":4,"ya":0,"za":0}}```

  Example:  
  ```xml
        <command name='rostopicread' topic='' expected='{"std_msg_String":{"data":"tosend"}}' repetitions='1' wait='0' reporterror='true'/>
  ```


-   **rostopicwrite**

    Write to ROS2 topic

    ROS2 supported types
    - std_msgs::msg::String  
      ```{"std_msg_String":{"data":"tosend"}}```

    - geometry_msgs::msg::Twist  
      ```{"geometry_msgs_Twist":{"x":1,"y":2,"z":3,"xa":4,"ya":0,"za":0}}```

Example:

```xml
    <command name='rostopicwrite' topic='' data='{"geometry_msgs_Twist":{"x":1,"y":2,"z":3,"xa":4,"ya":0,"za":0}}' repetitions='1' wait='0' reporterror='true'/>
```

# 4. Troubleshooting

- If tests don't work check ROS2 version and be sure that you have only one version installed.
- If ROS2 doesn't start check if you have executed the preliminary ROS2 bash script.
- Be careful with quotes and double quotes in tests.