#!/usr/bin/env python

"""
Date: 05/09/2019
Author: James Sohn

This is the master node of the package to subscribe to the measurement data publishded by publish_data.py and publishes the Robot state and Joint State to Gazebo simulator
"""
# import relevant libraries
import rospy
import numpy as np 
import tf
import math
import csv
from cp_simulator.msg import Sensor
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler

class Transform(object):
	def __init__(self):
		""" 
		initiate the parameters
		"""
		rospy.init_node('transform_data', anonymous=True)
		self._sensor_sub = rospy.Subscriber('sensor', Sensor, self.data_process)
		self._br = tf.TransformBroadcaster()
		self._rate = rospy.Rate(200.0)
		return

	def data_process(self, data):
		"""
		process the data and returns
		input:
			data - data class Sensor that contains acceleration and gyroscope data

		output:

		"""
		# assign variable from the sensor data
		self._acc_x = data.acc_x
		self._acc_y = data.acc_y
		self._acc_z = data.acc_z
		self._gyro_x = data.gyro_x
		self._gyro_y = data.gyro_y
		self._gyro_z = data.gyro_z

		# loginfo to check receiving data
		rospy.loginfo("Receiving data %f, %f, %f, %f" %(self._acc_x, self._acc_y, self._acc_z, self._gyro_x))

	def get_pose(self, filename):
		"""
		reads the pose from the .csv file
		"""
		with open(filename, mode='r') as csvreader:
			csv_reader = csv.reader(csvreader, delimiter=',')
			pos_time = list(csv_reader)

		return pos_time

	def tf_broadcast(self, pos):
		"""
		broadcasts tf data 
		"""
		# index
		ind = 0

		while not rospy.is_shutdown():
			#### checking block for conversion of string from .csv file to floating
			# rospy.loginfo("receiving %d" %(len(pos)))
			# rospy.loginfo("string %s" %(pos[ind][0]))
			# rospy.loginfo("float %f" %(float(pos[ind][0])))

			# step increase the index and rewind when maxed
			if ind == len(pos) - 1:
				ind = 0
				rospy.sleep(3)
			ind += 1

			# broadcast to tf
			self._br.sendTransform((float(pos[ind][0]), float(pos[ind][1]), float(pos[ind][2])), 
							(0.0, 0.0, 0.0, 1.0),
	                         rospy.Time.now(),
	                         "world",
	                         "chest")

			##### dummy broadcaster
			# t = rospy.Time.now().to_sec() * math.pi
			# self._br.sendTransform((2.0 * math.sin(t), 2.0 * math.cos(t), 0.0),
	  #                        (0.0, 0.0, 0.0, 1.0),
	  #                        rospy.Time.now(),
	  #                        "world",
	  #                        "chest")
			self._rate.sleep()

# main
def main():
	# input .csv file generated from the helper function sciprt
	filename = raw_input("> ")

	# instantiate the Transform class	
	transform = Transform()
	pos = transform.get_pose(filename)
	transform.tf_broadcast(pos)

	# # declare the subscriber node for sensor data
	# rospy.init_node('process_data', anonymous=True)

	# # initiate subscriber listening to topic sensor of type Sensor and calls data_process function
	# rospy.Subscriber('sensor', Sensor, data_process)

	# # set the communication rate to 200Hz compliant the sensor frequency
	# rate = rospy.Rate(1)

	# rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

