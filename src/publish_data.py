#!/usr/bin/env python

import rospy
from cp_simulator.msg import Sensor

def data_pub():
	# Initiate the node named 'data_publisher'
	rospy.init_node('data_publisher')

	# Publishing msg type 'Sensor' to topic 'sensor'. queue_size limits the number of queued messages if any subscriber is not receiving 
	pub = rospy.Publisher('sensor', Sensor, queue_size = 10)

	# Set the frequency of data transfer (sensor freq. 200Hz)
	rate = rospy.Rate(1)

	while not rospy.is_shutdown():
		# Instantiate the custom message
		data = Sensor()

		# Define the messages 
		data.acc_x = 1
		data.acc_y = 2
		data.acc_z = 3

		# logging for display
		rospy.loginfo("sending sensor data %d, %d, %d" %(data.acc_x, data.acc_y, data.acc_z))

		# publish data 
		pub.publish(data)

if __name__ == '__main__':
	try:
		data_pub()
	except rospy.ROSInterruptException:
		pass
