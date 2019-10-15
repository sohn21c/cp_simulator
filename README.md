## Cerebral Palsy Simulator  
#### _[James Sohn(Click to see the portfolio)](https://sohn21c.github.io)_  
#### _Apr 2019 ~ Jun 2019_ / _Oct 2019 ~ Dec 2019_  
#### _Last Updated: 10/14/19_
  
## Objective  
To successfully process and reproduce the human-body motion measured by the wireless IMU sensor(acceleartion and angular velocity) to 3-D simulation in Rviz, on ROS platform

## Background
The motivation of this project is to properly capture and reproduce the body motion of patients that are either diagnosed with Cerebral Palsy, or at risk of such diagnosis. The simulator will eventually be used to build machine learning model in an attempt to build the end-to-end system that can diagnose CP by having a personel wear sensors and take measurements.

## Software 
### Package versions
- Python 2.7.15+
- ROS Melodic 

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
From initial measurement of sensor and gravity represented in world frame(0, 0, -|gravity|), get RR.  
Sensor coordinate in world frame <- RR(dot)Identity matrix  
  
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

### Demo
Shown below is the intermediate demo of the software. A person is wearing two sensors, one on the forearm and the other on the upper arm.  
[![YouTube](https://github.com/sohn21c/cp_simulator/blob/master/pictures/demo_screenshot_1.png?raw=true)](https://youtu.be/aNzjvPvpOEo)  