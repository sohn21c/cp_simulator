#!/usr/bin/env python

"""
Date: 06/11/2019
Author: James Sohn

This node is to run two arm simulation in RVIZ
"""
# import relevant libraries
import rospy
import numpy as np 
import tf
import math
import csv
import matplotlib.pyplot as plt
from cp_simulator.msg import Sensor
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_matrix


class Transform(object):
	def __init__(self):
		""" 
		initiate the parameters
		"""
		rospy.init_node('two_arm_transform', anonymous=True)
		self._br = tf.TransformBroadcaster()
		self._rate = rospy.Rate(200.0)
		self._posx = []
		self._posy = []
		self._posz = []
		self._posRot = []

		return

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

		# # plot for cross-validate
		# plot_x = np.linspace(0, 100, len(pos_time))
		# plt.plot(plot_x, self._posx, 'r', label='x')
		# plt.plot(plot_x, self._posy, 'g', label='y')
		# plt.plot(plot_x, self._posz, 'b', label='z')
		# plt.title("Position", loc='center')
		# plt.legend(loc='upper left')
		# plt.ylabel("[m]")
		# plt.xlabel("Relative time")
		# plt.show()

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

			# conver angles to quaternion
			euler_x, euler_y, euler_z = euler_from_matrix(self._posRot[ind])
			posQuat = quaternion_from_euler(euler_x, euler_y, euler_z)

			# broadcast to tf
			# RVIZ axis flipped to real axis. All axis multiplied by (-1)
			self._br.sendTransform(
					(0., 0., 0.),	
					(0., 0., 0., 1.0), 
					rospy.Time.now(), 
					"upper_arm", 
					"base_link")
			self._br.sendTransform(
					(0.15, 0., 0.),	
					(posQuat[0], posQuat[1], posQuat[2], posQuat[3]), 
					rospy.Time.now(), 
					"elbow", 
					"upper_arm")

			# self._br.sendTransform(
			# 		(self._posx[ind]*5, self._posy[ind], self._posz[ind]),	
			# 		(posQuat[0], posQuat[1], posQuat[2], posQuat[3]), 
			# 		rospy.Time.now(), 
			# 		"upper_arm", 
			# 		"base_link")

			# rospy.loginfo("broadcasting %f %f %f %f" %(posQuat[0], posQuat[1],posQuat[2], posQuat[3]))

			# increment the index for next time step
			ind += 1

			# take rate in effect
			self._rate.sleep()

# main
def main():
	# instantiate the Transform class	
	transform = Transform()
	transform.get_pose("/home/james/catkin_ws/src/cp_simulator/src/two_arm1.csv")
	transform.tf_broadcast()

	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

