# "LINDO" The Home Assistant Robot [![Build Status](https://travis-ci.org/dwyl/esta.svg?branch=master)](https://travis-ci.org/)
This project introduces a home assistant mobile robot designed to be used for indoor navigation. It can be operated either autonomously or controlled by man remotely over the network. Odometry information is used to estimate the robot's position relative to its origin. In order to achieve a robust odometry, the robot uses two sources of odometry, the linear velocities from the encoders and the angular velocity from the IMU. A Kinect mounted on the robot creates a SLAM map with the help of edge detection and computer vision. The robot can navigate autonomously through the predefined map using AMCL. 

An assistant system is integrated on the robot allows it to communicate and socialize with humans, via a Chatbot powered by Artificial Intelligence and Machine Learning, connected to external APIs to bring realtime information. Also the robot can Automate and Secure your home via an IOT system. 

The project uses The Robots Operating System (ROS) which makes its functionality reusable in other projects. Modularity and readable codes are considered in the design and implementation of software nodes. About future work there is a wide field of updates like object detection, On-line Mapping of new Environments and installation of manipulator (i.e. robot arm) for a variet

## Rendered Designs:
#### Inner Frame:
<img src="https://github.com/LegendOfSparta/LINDO-The-Home-Assistant-Robot/blob/master/Media%20%26%20Docs/Media/Rendered/Solidworks%20Lindo%20Inner%20frame%20Design%201.png" />

#### Outer Frame:
<img src="https://github.com/LegendOfSparta/LINDO-The-Home-Assistant-Robot/blob/master/Media%20%26%20Docs/Media/Rendered/Solidworks%20Lindo%20Outer%20frame%20Design.png" />

## Hardware
- 4 Mecanum Wheels
- 4 Motors 12v 8.8 Kgf.cm
- 4 Encoders 3PPR
- 4 Channels Motor Driver
- Teensy 3.2
- Rasberry PI3
- MPU6050
- Kinect
- 12V Batteries

## Wiring Schematic:
<img src="https://github.com/LegendOfSparta/LINDO-The-Home-Assistant-Robot/blob/master/Media%20%26%20Docs/Media/Wiring%20Schematic.png" />

#### Notes:
- Fritizing doesn't have a 4 Channels Motor Driver yet, so we used an L298 Drivers on the schematic instead.

## Clearifying:
#### Master And Slave:
<img src="https://github.com/LegendOfSparta/LINDO-The-Home-Assistant-Robot/blob/master/Media%20%26%20Docs/Media/System%20Overview%20I.png"/>

#### Project Overview:
<img src="https://github.com/LegendOfSparta/LINDO-The-Home-Assistant-Robot/blob/master/Media%20%26%20Docs/Media/System%20Overview%20II.png" />

#### Rasberry PI(ROS) And Teensy:
<img src="https://github.com/LegendOfSparta/LINDO-The-Home-Assistant-Robot/blob/master/Media%20%26%20Docs/Media/System%20Overview%20III.png" />

## Platform:

#### Development Labtop:
- ROS Kinetic (Ubuntu 16.04 Xenial)

#### Rasberry Pi 3:
- ROS Kinetic (Ubuntu Mate 16.04)

## User Manual:
#### Creating a Map:
- On robot’s computer,  Launch base driver:
```
roslaunch lindorobot bringup.launch
```
- On robot’s computer, Launch mapping packages:
```
roslaunch lindorobot slam.launch
```
- On your development computer, run teleop_twist_keyboard:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py	
```
- On your development computer, Run rviz:
```
roscd lindo_visualize/rviz
rviz -d slam.rviz
```
- Once you are done mapping save the map by running map_server on the robot's computer:
```
rosrun map_server map_saver -f ~/lindorobot_ws/src/linodorobot/maps/map
```

#### Autonomous Navigation
- Launch base driver, on robot’s computer:
```
roslaunch lindorobot bringup.launch
```
- Launch navigation packages, on robot’s computer:
```
roslaunch lindorobot navigate.launch
```
- On your development computer, run rviz:
```
roscd lindo_visualize/rviz
rviz -d navigate.rviz
```


## Assistant System:
#### Overview Of the Assistant System:
<img src="https://github.com/LegendOfSparta/LINDO-The-Home-Assistant-Robot/blob/master/Media%20%26%20Docs/Media/Assistant%20System%20Overview.png" />

#### DialogFlow Overview:
<img src="https://github.com/LegendOfSparta/LINDO-The-Home-Assistant-Robot/blob/master/Media%20%26%20Docs/Media/Dialog%20Flow.png" />

## Videos
- <a href="https://www.youtube.com/watch?v=ZyevQ8gZiK0">"LINDO" - Teleopration Intial Tests[Before PID Tunning]</a>
- <a href="https://www.youtube.com/watch?v=e5AFlajBlEw">"LINDO" - Localization Intial Tests Using IMU and Encoders</a>
- <a href="https://www.youtube.com/watch?v=WcR59p794sQ">"LINDO" - Depth Generation Intial Tests Using Kinect</a>
- <a href="https://www.youtube.com/watch?v=mfHSG0Qb3YM">"LINDO" - Autonomus Driving and Obstacle Avoidance Intial Test </a>
- <a href="https://www.youtube.com/watch?v=wUoYN-ng8Ps">"LINDO" - Assistant System V1 [DialogFlow+Firebase+Database+NodeMCU+APIs]</a>
- <a href="https://www.youtube.com/watch?v=nZiiJRJRGrw">"LINDO" - Assistant System Voice-Ready Intial Test</a>

## TEAM AWAKEN:
#### Members:
- Mohamed Abdo
- Mahmoud Mahmoud
- Islam El-Nagdy
- Ahmed Hamed

#### From:
- MUST University,
- Engineering Faculty,
- Mechatronics Department

#### Contact Us:
- Email: moha_the.best@hotmail.com
