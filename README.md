## Cerebral Palsy Simulator  
#### _[James Sohn(Click to see the portfolio)](https://sohn21c.github.io)_  
#### _Apr 2019 ~ Jun 2019_  
  
## Objective  
This project is to build RVIZ simulator that would display robot model of a patient wearing 5 IMU sensors. Completed simulator can read IMU sensor readings(acceleration and angular velocity) in .tsv format, perform proper dead-reckoning and visualize position and orientation of five sensors in 3-d space.

## Project timeline
One can find the detailed project timeline [here](https://github.com/sohn21c/cp_simulator/blob/master/schedule.md)  

## Nodes / Helper function scripts  
- _transform_data.py(node)_:  
	takes in position and so3 matrix components and broadcasts to tf topic 
	Input: .csv file. Position and so3 matrix components 
	Ouput: tf msg  

- _data_parser.py(helper function script)_:  
	imports data from the sensor measurements in .tsv format and parses them into individual containers
	Input: filename  
	Ouput: 6 lists (acc, ang_vel)  

- _computation.py(helper function script)_:  
	computes position and orientation of sensor frame in 3d and saves in .csv file
	Input: acc, ang_vel  
	Output: pos, so3 matrix  


## Algorithm  
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

