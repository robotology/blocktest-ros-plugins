# 1. Test summary
This test is designed for testing ROS2 applications.

# 2. Prerequisites
- [Robotology-superbuild](https://github.com/robotology/robotology-superbuild) or just YCM
- [Blocktest](https://github.com/robotology/blocktest)
- [Blocktest-ros](https://github.com/robotology/blocktest-ros-plugins)
- Install ROS2 foxy

# 3. Execution

Base test,
the test will send and receive messages through the ROS2 topic message system used topic=`mytopic`. 
```bash
cd <your-blocktest-clonedir>/blocktest/install/bin
./blocktestrunner simple001.xml tests/test-ros2-simple
```

The following thest will fail,
the test will try to receive messages through the ROS2 topic message system used topic=`mytopic` but fails. 
```bash
cd <your-blocktest-clonedir>/blocktest/install/bin
./blocktestrunner failreceive.xml tests/test-ros2-simple
```

