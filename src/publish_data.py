#!/usr/bin/env python
"""
Date: 05/08/2019
Author: James Sohn

This node reads the measurement data from the .csv file from the sensor and publishes the data on the Sensor topic
"""
# import relavant libraries
import rospy
import time
import data_parser
from cp_simulator.msg import Sensor
from gemetry_msgs.msg import Transform

# define data_publisher
def data_pub(filename):
	# retrieve the data from the data_parser module
	acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = data_parser.data_parser(filename)
	# create the data index 
	ind = 0
	# initiate the ROS node named 'data_publisher'
	rospy.init_node('publish_data')
	# publishing msg type 'Sensor' to topic 'sensor'. queue_size limits the number of queued messages if any subscriber is not receiving 
	pub_sensor = rospy.Publisher('sensor', Sensor, queue_size = 10)
	# set the frequency of data transfer (sensor freq. 200Hz)
	############ Rate control not working!!!
	rate = rospy.Rate(1)
	##### delay 5 seconds
	time.sleep(5)
	##### to measure the duration of the loop
	current_time = rospy.Time.now()

	while not rospy.is_shutdown():
		# instantiate the custom message
		data = Sensor()
		# slice the data points 
		data.acc_x = acc_x[ind]
		data.acc_y = acc_y[ind]
		data.acc_z = acc_z[ind]
		data.gyro_x = gyro_x[ind]
		data.gyro_y = gyro_y[ind]
		data.gyro_z = gyro_z[ind]
		# increase the data index by 1
		ind += 1

		# rewind the index when reaching at the end of the data set
		if ind == len(acc_x):
			ind = 0

			##### measure time to find the publish rate
			end_time = rospy.Time.now()
			print((end_time.nsecs - current_time.nsecs) / 10**9)

			# pause the loop at the end of the data set
			rospy.sleep(5)
			
			##### reset the time tick
			current_time = rospy.Time.now()

		# logging for display
		print(data.gyro_x)
		# print(ind)
		# rospy.loginfo("sending sensor data %d, %d, %d" %(data.acc_x, data.acc_y, data.acc_z))

		# publish data 
		pub_sensor.publish(data)

if __name__ == '__main__':
	try:
		# receive command line input for file name
		filename = input("> ")
		# publishes data from the file
		data_pub(filename)

	except rospy.ROSInterruptException:
		pass
