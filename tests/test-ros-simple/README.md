# 1. Test summary
This test is designed for testing Mc4Plus board.

# 2. Prerequisites
- [Robotology-superbuild](https://github.com/robotology/robotology-superbuild)
- [Blocktest](https://github.com/robotology/blocktest)
- [Blocktest-yarp](https://github.com/robotology/blocktest-yarp-plugins)

The mc4plus board connected to the [setup](https://github.com/icub-tech-iit/fix/issues/926) with the motor and  `AEA2` encoder.

# 3. Execution
```bash
cd <your-blocktest-clonedir>/blocktest/install/bin
./blocktestrunner test.xml test-mc4plus/
```

# 4. Test description
The test will start the `yarpserver` and then it will perform continuosely the following sequence actions:
1. Start `yarprobotinterface`

2. Open the driver resources

3. Actuates in position control mode the motor connected with `mc4plus` following the degree pattern: `[30 -30 60 -60 -120 -120 145 -145]`.

5. Close the driver resources.

6. Check for the desired `error string` and eventually create a new `.log` backup file.
