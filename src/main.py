#!/usr/bin/env python

"""
Date: 05/09/2019
Author: James Sohn

This is the master node of the package to subscribe to the measurement data publishded by publish_data.py and publishes the Robot state and Joint State to Gazebo simulator
"""
# import relevant libraries
import rospy
from cp_simulator.msg import Sensor
import numpy as np 

# set the conversion constants for the sensor data
fps = 200
acc_conv = 8 / 2**16
gyro_conv = 1000 / 2**16 * np.pi / 180

# data processing
def data_process(data):
	"""
	process the data and returns
	input:
		data - data class Sensor that contains acceleration and gyroscope data

	output:

	"""
	# assign variable from the sensor data
	acc_x = data.acc_x
	acc_y = data.acc_y
	acc_z = data.acc_z
	gyro_x = data.gyro_x
	gyro_y = data.gyro_y
	gyro_z = data.gyro_z

	# loginfo to check receiving data
	rospy.loginfo("Receiving data %f, %f, %f, %f" %(acc_x, acc_y, acc_z, gyro_x))

# main
def main():

	# declare the subscriber node for sensor data
	rospy.init_node('sensor_listener', anonymous=True)
	# initiate the subscriber node
	# subscribing to topic sensor of type Sensor and calls data_process function
	rospy.Subscriber('sensor', Sensor, data_process)

	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass