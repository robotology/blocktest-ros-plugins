# 1. blocktest-ros-plugins


- [1. blocktest-ros-plugins](#1-blocktest-ros-plugins)
- [2. Installation](#2-installation)
  - [2.1. Prerequisite Linux](#21-prerequisite-linux)
  - [2.2. Installation and compilation](#22-installation-and-compilation)
- [3. Available commands](#3-available-commands)
  - [3.1. ROS commands](#31-ros-commands)


Repository containing the ros plugins for blocktest


# 2. Installation

Supported OS: Linux Ubuntu 20.04.
Boost library version must be >1.64.


## 2.1. Prerequisite Linux

```bash
sudo apt-get install -y cmake libboost-all-dev
```
ROS2 see https://docs.ros.org/en/rolling/Installation.html


## 2.2. Installation and compilation

In order to compile just execute the following commands in a bash
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

    ```xml
        <command name="rostopicread" topic="" expected="" repetitions="1" wait="0" reporterror="true"/>
    ```


-   **rostopicwrite**

    Write to ROS2 topic

    ```xml
        <command name="rostopicwrite" topic="" data="" repetitions="1" wait="0" reporterror="true"/>
    ```