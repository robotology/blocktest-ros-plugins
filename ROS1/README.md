# 1. blocktest-ros1-plugins

Repository containing the ROS1 plugins for blocktest


# 2. Installation

Supported OS: Linux Ubuntu 20.04.
Boost library version must be >1.64.


## 2.1. Prerequisite Linux

Install the following:
```bash
sudo apt-get install -y cmake libboost-all-dev
```

### 2.1.1. ROS1
**ROS1 Noetic** version see http://wiki.ros.org/noetic/Installation

### 2.1.2. Blocktest
See https://github.com/robotology/blocktest


## 2.2. Installation and compilation

To compile just execute the following commands in a bash
terminal.
```bash
git clone https://github.com/robotology/blocktest-ros1-plugins     
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
These action blocks are contained in blocktest-ros1-plugin.

## 3.1. ROS commands

-   **ros1topicread**
todo

-   **rostopicwrite**

todo


# 4. Troubleshooting

- If tests don't work check ROS1 version and be sure that you have only one version installed.
- If ROS1 doesn't start check if you have executed the preliminary ROS1 bash script.
- Be careful with quotes and double quotes in tests.

# 5. Debug tools

## 5.1. Subscribe to topics with cmdline

`rostopic echo /topic_name`

todo
`ROS1 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"`

# 6. Warnings

ROS1 needs `roscore` to be executed
