1. the impedance controller: the controller is ok
2. the force sensor related: the force sensor is ok
3. the IMU sensor related: 
4. the AUBO5 robot arm: the ros driver is ok

debugging log on 20200703 afternoon:
1. the force sensor is connected and the data is well received, and the command is:
rosrun robotiq_ft_sensor rq_sensor
rosrun robotiq_ft_sensor rq_test_sensor

2. the IMU sensor is connected and the data is well received, and the command is:
roslaunch sensor_startup imu_bringup.launch 
rostopic echo /imu

3. the  AUBO ros driver

debugging log on 20200703 night:
1. the USB port of IMU should be plugged in firstly and then the robotiq force sensor USB port should be plugged in 
2. two USB ports should open:
ls /dev/ttyUSB*
chmod 777 /dev/ttyUSB*
3. roslaunch polishing_undervibration test1.launch 
which aims to run aubo5 arm, imu and robotiq force sensor 








