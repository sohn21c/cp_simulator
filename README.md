# Human Motion Simulator with Wearable IMU Sensor
#### _[James Sohn(Click to see the portfolio)](https://sohn21c.github.io)_  
  
## Objective  
The purpose of the software shared in the repo is to process measurements of the human motion collected by the group of wireless IMU sensors(acceleration and angular velocity) and reproduce such motiones represented by the numerical data points as 3-D simulation of humanoid model in Rviz, on ROS platform

## Contents
1. [Introduction](#introduction)
2. [Background Information](#background-information)
3. [Sensor](#sensor)
4. [Software](#software)
5. [Challenges](#challenge)
6. [Demo](#demo)
7. [Citation](#citation)

## Introduction
The inception of this software stems from an idea to create a ML model that can help diagnose the physical challenge, Cerebral Palsy by measuring human subject's motion with wearable IMU sensors, capable of capturing linear acceleartion and angular velocity in 3 orthogonal axis. Being a disease that hinders the motorized movement of subject's body, sensors attached to subject's limbs are to be able to properly caputre the characteristic of human motion. Aligned with such intention, the creation of simulation software that can reproduce the recorded motion of human body in virtual environment is a verification tool for the validity of the measurements. It can be further utilized to allow the conversion of human motion represented in linear accleration and angular velocity to characterize critical patterns of motion contributing to the diagnosis of Cerebral Palsy.  

## Background Information
1. Dead Reckoning  
	The technique, estimating pose and orientatin of a sensor in the 3-d space, is known as dead reckoning. What the software, introduced in this page, does to reproduce human motion in simulation environment is essentially the same. The software has to be able to compute the update of pose and estimation of each sensor, processing the linear acceleration and angular velocity. One can find more information about dead reckoning [in this wiki page.](https://en.wikipedia.org/wiki/Dead_reckoning)  

2. Robotic Operating System(ROS)  
	The ROS is a software framework that, at its core, offers a message passing interface that provides inter-process communication and is commonly referred to as a middleware. ROS is a natural choice platform for the introduced software as it'd need a platform to process a series of data in parallel and consolidate the update in visual format. One can find more information about ROS [here.](https://www.ros.org/core-components/)
	
3. Simultaneous Orthogonal Rotation Angle  
	Due to the nature of the challenge to estimate the orientation of the sensor at next time step, based on current measurement of angular velocity at current time step in 3-D, sequential Euler rotation in each axis introduces systematic error as the rotation is non-commutative. Such requirement calls for simultaneous computation of rotation. One can find more information about Simultaneous Orthogonal Rotation Angle [here.](https://www.hindawi.com/journals/js/2018/9684326/)

## Sensor  
The sensor is encapsulated inside flexible silicone material containing IMU sensors. The sensor is capable of wireless charging and data transporation via bluetooth.  
- IMU sensor  
	BMI 160 Bosch Sensortec IMU  
	[Data Sheet](https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BMI160-DS000.pdf)  
- Device  
	Sensor encapsulation and entire development of mechanical and electrical components of the sensor is performed and provided by [Roges Research Group at Northwestern University](http://rogersgroup.northwestern.edu/)   
<img src="https://github.com/sohn21c/cp_simulator/blob/master/img/sensor_on_body.png?raw=true" alt="Sensor on body" width="300">  

## Software 
### Package versions
- Python 2.7.15+
- ROS Melodic 

### Package Structure
```
root: ~/catkin_ws/src/cp_simulator/
  launch/ 	: contains launch file that initiates the nodes
  rviz/		: contains .rviz file with configured setting
  src/		: contains python node scripts
  urdf/		: contains Universal Robot Description Files 
```

### Nodes / Helper function scripts  
- _upper_body_transform.py(node)_:  
	takes in position and so3 matrix components and broadcasts to tf topic in quaternion angle  
	Input: .csv file(pos, so3 matrix)  
	Ouput: ROS tf msg  

- _data_parser.py(helper function script)_:  
	imports data from the sensor measurements in .tsv format and parses them into individual containers of 3-axis accleration and 3-axis angular velocity
	Input: filename(.tsv)  
	Ouput: 6 lists (3 acc, 3 ang_vel)  

- _computation.py(helper function script)_:  
	computes position and orientation of sensor frame in 3d environment and saves in .csv file
	Input: 6 lists (3 acc, 3 ang_vel)  
	Output: pos, so3 matrix  

### Algorithm  
```
From initial measurement of sensor and gravity represented in world frame({w}) (0, 0, -|gravity|), get RR(rotation mat).  
Sensor coordinate({b}) in world frame <- RR(dot)Identity matrix  
  
While not done:  

	sensor acceleration in {w} <- RR(dot)sensor acceleration in {b}  
	sensor acceleration in {w} -= gravity  
	velocity in {w} = sensor acceleration in {w} * T  
	position in {w} = velocity in {w} * T  
	rotation of frame = Single Orthogonal Rotation Angle(ang velocity)  
	updated sensor coordinate = sensor coordinate(dot)rotation of frame  
	updated RR = RR(dot)rotation of frame  
	get quaternion from so3  

return position in {w}, quaternion   
```

## Challenge  
During the development of the software, a number of technical challenges were encountered and addressed.  

### Dilemma of Dead Reckoning  
It is a well known issue that dead reckoning, estimating position and orientation of an object, integrating the linear acceleration and angular velocity is inherently very prone to accumulation of error. There are different approaches to address the issue but it tends to involve external devices such as magnotometer or GPS. However, specific to the scope of the application, pursued by introduced software, estimation of the position and orientation can be calculated by constraining the robot model to the designed geometry. Instead of updating the position of sensor in the 3d environment by the double integration of linear acceleartion, only the orientation of the sensor is calculated by single integration of angular velocity. Body model, constrained to the URDF model, can only have one available position at the given orientation.  

### Sensor Bias and Noise
IMU sensors carry DC bias and noise. Both were observed from plotting the trials with stationary sensor measurements as shown in the figure below. Although DC bias can be manually balanced by element-wise subtracting the individual bias values from the measurement data, it can also be addressed by conducting sensor calibration on firmware level.  
Bias is represented as distance of solid lines for each axis of measurement from absolute zero in y-axis, and noise is represented as high frequency variation of values.  
<img src="https://github.com/sohn21c/cp_simulator/blob/master/img/bias_noise.png?raw=true" alt="Sensor on body" width="800"> 

### Signal Processing
<img src="https://github.com/sohn21c/cp_simulator/blob/master/img/stationary_sensor.png?raw=true" alt="Sensor on body" width="300">  
<img src="https://github.com/sohn21c/cp_simulator/blob/master/img/moving_sensor.png?raw=true" alt="Sensor on body" width="300">  
Shown above are FFT plots of measurement of single stationary sensor and single moving sensor. First plot shows, as confirmed in the previous section, the bias signal appearing at 0Hz(DC bias). Second plot captures the FFT plot of frequency range of human motion. One can find detailed research in other scholarly articles that supports the observation that relevant frequency range of human motion is between 0 - 20 Hz. Author integrated desgned Low Pass Filter filtering out the noise of frequency higher than 20Hz. 

## Demo
### Proof-of-concept
Shown below is the intermediate demo of the software. A person is wearing two sensors, one on the forearm and the other on the upper arm.  
[![YouTube](https://github.com/sohn21c/cp_simulator/blob/master/pictures/demo_screenshot_1.png?raw=true)](https://youtu.be/aNzjvPvpOEo)  

### Upper body

### Full body

## Citation
```
@misc{Human Motion Simulator with Wearable IMU Sensor,
    author       = {James Sohn},
    title        = {{Human Motion Simulator with Wearable IMU Sensor}},
    month        = dec,
    year         = 2019,
    version      = {1.0},
    url          = {https://github.com/sohn21c/cp_simulator/}
    }
```
