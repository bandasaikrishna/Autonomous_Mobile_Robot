## Autonomous Navigation Mobile Robot using ROS Navigation Stack.
Watch the video tutorial [here .](https://youtu.be/Uz_i_sjVhIM)
Website documentation [here.](https://www.rosroboticslearning.com/)

AMCL is used for localisation, with DWA local planner planner for path planning.

### Sensors used:
1. RP Lidar A1-M8.
2. Motor Encoders.

### Other parts:
1. Nvidia Jetson Nano 4gb.
2. 2x Arduino nano (One for each motor to make sure smooth operation of Interrupt service routine to count the encoder pulses and ROS communication over I2C.)
3. Encoder motors [Link Here](https://www.youtube.com/redirect?redir_token=QUFFLUhqbW04YmUxb0dZdUQwUXpzM2RQY0w4bk9UV3NwZ3xBQ3Jtc0ttVzBTWkRtazZCd29jejhZNldUM3ZOczVJdHFJaWhxVTNTTzVtUWxRem8zZW9xSFc3RHVaMlNrTHZSTlh0dmNaeEdTOEExOVR4X0lmTmJKWlFaOGhwLVhBcG9pMXlYSlk1Si1CV0pTV0JjSHE0MGx4MA%3D%3D&q=https%3A%2F%2Frobokits.co.in%2Fmotors%2Frhino-gb37-12v-dc-geared-motor%2Fdc-12v-encoder-servo-motors%2Frhino-gb37-12v-60rpm-10.4kgcm-dc-geared-encoder-servo-motor&stzid=Ugwdoo-96VupDTGEd4t4AaABAg.9GsOUycHAnS9GsgP_3zflw&event=comments)
4. Robot Chasis
5. Boost converter to provide 12V for motors.

![Jetson Nano and LiDAR](https://github.com/0xraks/Autonomous_Mobile_Robot/raw/main/mobile_robot_autonomous_navigation/2020-12-07%2011_36_51-Autonomous%20Navigation%20Mobile%20Robot%20using%20ROS%20_%20Jetson%20Nano%20_%20RPLidar%20_%20Different.png)
![Arduinos and motors](https://github.com/0xraks/Autonomous_Mobile_Robot/raw/main/mobile_robot_autonomous_navigation/2020-12-07%2011_37_10-Autonomous%20Navigation%20Mobile%20Robot%20using%20ROS%20_%20Jetson%20Nano%20_%20RPLidar%20_%20Different.png)





