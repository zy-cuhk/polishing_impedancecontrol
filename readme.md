1. the impedance controller: the controller is ok
2. the force sensor related: the force sensor is ok
3. the IMU sensor related: 
4. the AUBO5 robot arm: the ros driver is ok

debug log:
1. the force sensor is connected and the data is well received, and the command is:
rosrun robotiq_ft_sensor rq_sensor
rosrun robotiq_ft_sensor rq_test_sensor

2. the IMU sensor is connected and the data is well received, and the command is:
roslaunch sensor_startup imu_bringup.launch 
rostopic echo /imu

3. the  AUBO ros driver



