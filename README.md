# Implementing a PID controller to allow wall following while mapping the space!

https://github.com/khushant2001/Irobot_room_mapping/assets/70731991/6eef20a8-ffd1-4ec5-879d-03e078c9682e

For this project, Robotic Operating System (ROS 2, distro = 'humble') is used to implement room mapping through Irobot Create. The documentation for the Irobot Create ROS structure can be found [here](https://iroboteducation.github.io/create3_docs/) and the Ros2 documentation can be found [here](https://docs.ros.org/en/humble/index.html). A traditional pub sub method is used to communicate with the Irobot. Following is the list of topics used to communicate with the robot:

### Subscribed

- DockStatus: irobot_create_msgs.msg // checking if the Irobot is docked
- HazardDetectionVector: irobot_create_msgs.msg // get a list of hazards 
- IrIntensityVector: irobot_create_msgs.msg // get the reading from the 7 IR sensors
- Imu: sensor_msgs.msg // get the X and Y coordinate of the Irobot

### Published

- Twist: geometry_msgs.msg // send the velocity commands to the Irobot

All of these communications occur through the designed node. Rqt graph of the pub-sub structure used is shown below: 

## 1. Designing the controller for wall following:

### I. Bang - Bang controller

For the first try, a simple bang bang controller is designed which takes in the Ir sensor readings and descides the angular velocity based on hardcoded limits on the intensities. The implementation is done in the "test" node which can be run by doing "ros2 run controller new". Following is the algorithm put to use:

- Get the IR sensor readings.
- Mode 1: If the max intensity of 3 IR sensors from left is higher than 100, move closer to the wall with anglular velocity of 0.2.
- Mode 2: If the max intensity of leftmost sensor is less than 10, move closer to the wall with angular velocity of 0.4.
- Mode 3: If any IR reading is greater than 350, find the respective sensor. If the sensor if in the left [0:3] then move to the right with angular velocity of -0.6.

Despite working properly, the controller isnt't precise at all and is extremely slow. Increasing the linear speed would mess up the logic. Accordingly, its not adequate. 

### II. PID controller

For the second try, a PID controller is implemented to keep the intensity of the leftmost IR sensor at 70 (setpoint). Accordingly, the error is determined and is processed as follows: 

$$ 
u = K_p * error + K_i * \int error \dt + K_d * \frac{d(error)}{dt}
$$

The gains are tuned manually using the 
