# 1. Test summary
This test is designed for testing ROS2 applications.

# 2. Prerequisites
- [Robotology-superbuild](https://github.com/robotology/robotology-superbuild) or just YCM
- [Blocktest](https://github.com/robotology/blocktest)
- [Blocktest-ros](https://github.com/robotology/blocktest-ros-plugins)
- [robometry](https://github.com/robotology/robometry)(optional)
- Install ROS2 foxy

# 3. Execution

## 3.1 Base test
The test will send and receive messages through the ROS2 topic message system used topic=`mytopic`. 
```bash
cd <your-blocktest-clonedir>/blocktest/install/bin
./blocktestrunner simple001.xml tests/test-ros2-simple
```

## 3.2 Fast test
The test will send and receive messages through the ROS2 topic message system used topic=`mytopic`. 
```bash
cd <your-blocktest-clonedir>/blocktest/install/bin
./blocktestrunner fast.xml tests/test-ros2-simple
```

## 3.3 Fail receive
The following thest will fail,
the test will try to receive messages through the ROS2 topic message system used topic=`mytopic` but fails. 
```bash
cd <your-blocktest-clonedir>/blocktest/install/bin
./blocktestrunner failreceive.xml tests/test-ros2-simple
```
## 3.4 Robometry(needs `robometry` installed and enabled)
The test will launch a jointState publisher and a robometry blockt in paraller. The robometry block dump the data received in mat files.
```bash
cd <your-blocktest-clonedir>/blocktest/install/bin
./blocktestrunner robometry.xml tests/test-ros2-simple
```

