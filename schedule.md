## Projected Schedule  
#### Milestones  
1. ROS software structure build with Rviz  
	To build necessary nodes that can transfers appropriate information from each other for future dead-reckoning computation and visualization in RVIZ.  
2. Come up with the algorithm for dead-reckoning  
	Strategize how to properly decide position and orientation of the sensor frame in 3d from acceleration and angular velocity.  

3. Algorithm implementation  
	Implement the algorithm on the ROS platform for visualization  

4. Verification with demo  
	Verify the result with the planned test and demonstration with the sensor  

5. (if necessary) Noise filtering/Signal Processing  
	Research different methods to reduce the noise and error for double-integration for dead-reckoning  

#### Weekly goal  
- Week of 04/15:
	- Kick-off meeting - Discuss the scope of the project  


- Week of 04/22:
	- Get to know sensors used in the device and understand the data format  

- Week of 04/29:
	- Write a script(parser.py) that parses .tsv file and collects data in to individual container for future computation.  

- Week of 05/06:
	- Write base ROS structure for visualization in Rviz  

- Week of 05/13:
	- Write Algorithm for dead-reckoning  

- Week of 05/20:
	- Algorithm verification  
	- Implement the algorithm into a number of nodes on ROS platform  

- Week of 05/27:
	- Verify the implementation with the trial  

- Week of 06/03:
	- Debuggin and finalize the project  