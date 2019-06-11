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
import matplotlib.pyplot as plt
import tf.transformations as tr 
from cp_simulator.msg import Sensor
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_matrix


class Transform(object):
	def __init__(self):
		""" 
		initiate the parameters
		"""
		rospy.init_node('transform_data', anonymous=True)
		self._sensor_sub = rospy.Subscriber('sensor', Sensor, self.data_process)
		self._br = tf.TransformBroadcaster()
		self._rate = rospy.Rate(20.0)
		self._posx = []
		self._posy = []
		self._posz = []
		self._posRot = []

		return

	def data_process(self, data):
		"""
		%%% NOT USED %%%
		process the data and returns
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

			# parse the pose.csv file and separate the pose information. All data point convered to floating number from csv
			for i in range(len(pos_time)):
				self._posx.append(float(pos_time[i][0]))
				self._posy.append(float(pos_time[i][1]))
				self._posz.append(float(pos_time[i][2]))
				self._posRot.append(np.array([[float(pos_time[i][3]), float(pos_time[i][4]), float(pos_time[i][5])], [float(pos_time[i][6]), float(pos_time[i][7]), float(pos_time[i][8])], [float(pos_time[i][9]), float(pos_time[i][10]), float(pos_time[i][11])]]))

		# plot for cross-validate
		plot_x = np.linspace(0, 100, len(pos_time))
		plt.plot(plot_x, self._posx, 'r', label='x')
		plt.plot(plot_x, self._posy, 'g', label='y')
		plt.plot(plot_x, self._posz, 'b', label='z')
		plt.title("Position", loc='center')
		plt.legend(loc='upper left')
		plt.ylabel("[m]")
		plt.xlabel("Relative time")
		plt.show()

	def tf_broadcast(self):
		"""
		broadcasts tf data 
		"""
		# index
		ind = 0

		while not rospy.is_shutdown():
			# ### checking block for conversion of string from .csv file to floating
			# rospy.loginfo("receiving %d" %(len(pos)))
			# rospy.loginfo("string %s" %(pos[ind][0]))
			# rospy.loginfo("float %f" %(float(ops[ind][0])))

			# step increase the index and rewind when maxed
			if ind == len(self._posx) - 1:
				ind = 0
				rospy.sleep(3)

			# convert angles to quaternion
			# posQuat = so3_to_quat(self._posRot[ind])

			##### conver angles to quaternion
			euler_x, euler_y, euler_z = euler_from_matrix(self._posRot[ind])
			posQuat = quaternion_from_euler(euler_x, euler_y, euler_z)

			# broadcast to tf
			# RVIZ axis flipped to real axis. All axis multiplied by (-1)
			self._br.sendTransform(
					(self._posx[ind]*5, self._posy[ind], self._posz[ind]),	
					(posQuat[0], posQuat[1], posQuat[2], posQuat[3]), 
					rospy.Time.now(), 
					"chest", 
					"world")

			# #### test broadcaster for rotation
			# # RVIZ axis flipped to real axis. All axis multiplied by (-1)
			# self._br.sendTransform(
			# 		(0.0, 0.5, 0.0),	
			# 		(posQuat[0], posQuat[1], posQuat[2], posQuat[3]), 
			# 		rospy.Time.now(), 
			# 		"chest", 
			# 		"world")

			# ##### test broadcaster for translation
			# # RVIZ axis flipped to real axis. All axis multiplied by (-1)
			# self._br.sendTransform(
			# 		(self._posx[ind], self._posy[ind], self._posz[ind]),	
			# 		(0., 0., 0., 1.0), 
			# 		rospy.Time.now(), 
			# 		"chest", 
			# 		"world")

			rospy.loginfo("broadcasting %f %f %f %f" %(posQuat[0], posQuat[1],posQuat[2], posQuat[3]))

			# increment the index for next time step
			ind += 1
			self._rate.sleep()

			# #### dummy broadcaster
			# t = rospy.Time.now().to_sec() * math.pi
			# self._br.sendTransform((2.0 * math.sin(0.1*t), 2.0 * math.cos(0.1*t), 0.0),
	  #                        (0.0, 0.0, 0.0, 1.0),
	  #                        rospy.Time.now(),
	  #                        "world",
	  #                        "chest")

			# #### dummy broadcaster
			# q = quaternion_from_euler(1.5707, 1.5707, 3.14)
			# self._br.sendTransform((1.0, 0.0, 0.0),
	  #                        (q[0], q[1], q[2], q[3]),
	  #                        rospy.Time.now(),
	  #                        "chest",
	  #                        "world")


# main
def main():
	# input .csv file generated from the helper function sciprt
	filename = raw_input("> ")

	# instantiate the Transform class	
	transform = Transform()
	transform.get_pose(filename)
	transform.tf_broadcast()

	# # declare the subscriber node for sensor data
	# rospy.init_node('process_data', anonymous=True)

	# # initiate subscriber listening to topic sensor of type Sensor and calls data_process function
	# rospy.Subscriber('sensor', Sensor, data_process)

	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

