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
		self._posx1 = []
		self._posy1 = []
		self._posz1 = []
		self._posRot1 = []
		self._posx2 = []
		self._posy2 = []
		self._posz2 = []
		self._posRot2 = []

		return

	def get_pose(self, filename1, filename2):
		"""
		reads the pose from the .csv file
		"""
		with open(filename1, mode='r') as csvreader:
			csv_reader = csv.reader(csvreader, delimiter=',')
			pos_time = list(csv_reader)

			# parse the pose.csv file and separate the pose information. All data point convered to floating number from csv
			for i in range(len(pos_time)):
				self._posx1.append(float(pos_time[i][0]))
				self._posy1.append(float(pos_time[i][1]))
				self._posz1.append(float(pos_time[i][2]))
				self._posRot1.append(np.array([[float(pos_time[i][3]), float(pos_time[i][4]), float(pos_time[i][5])], [float(pos_time[i][6]), float(pos_time[i][7]), float(pos_time[i][8])], [float(pos_time[i][9]), float(pos_time[i][10]), float(pos_time[i][11])]]))

		with open(filename2, mode='r') as csvreader:
			csv_reader = csv.reader(csvreader, delimiter=',')
			pos_time = list(csv_reader)

			# parse the pose.csv file and separate the pose information. All data point convered to floating number from csv
			for i in range(len(pos_time)):
				self._posx2.append(float(pos_time[i][0]))
				self._posy2.append(float(pos_time[i][1]))
				self._posz2.append(float(pos_time[i][2]))
				self._posRot2.append(np.array([[float(pos_time[i][3]), float(pos_time[i][4]), float(pos_time[i][5])], [float(pos_time[i][6]), float(pos_time[i][7]), float(pos_time[i][8])], [float(pos_time[i][9]), float(pos_time[i][10]), float(pos_time[i][11])]]))

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
			if ind == len(self._posx1) - 1:
				ind = 0
				rospy.sleep(3)

			# conver angles to quaternion
			euler_x, euler_y, euler_z = euler_from_matrix(self._posRot1[ind])
			posQuat1 = quaternion_from_euler(euler_x, euler_y, euler_z)

			euler_x, euler_y, euler_z = euler_from_matrix(self._posRot2[ind])
			posQuat2 = quaternion_from_euler(euler_x, euler_y, euler_z)
			# broadcast to tf
			# RVIZ axis flipped to real axis. All axis multiplied by (-1)
			self._br.sendTransform(
					(0., 0., 0.),	
					(posQuat1[0], posQuat1[1], posQuat1[2], posQuat1[3]), 
					rospy.Time.now(), 
					"upper_arm", 
					"base_link")
			self._br.sendTransform(
					(0.15, 0., 0.),	
					(posQuat2[0], posQuat2[1], posQuat2[2], posQuat2[3]), 
					rospy.Time.now(), 
					"elbow", 
					"upper_arm")

			# rospy.loginfo("broadcasting %f %f %f %f" %(posQuat[0], posQuat[1],posQuat[2], posQuat[3]))

			# increment the index for next time step
			ind += 1

			# take rate in effect
			self._rate.sleep()

# main
def main():
	# instantiate the Transform class	
	# filename = raw_input("filename > ")
	filename1 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "two_arm8_upper.csv"
	filename2 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "two_arm8_fore.csv"
	transform = Transform()
	transform.get_pose(filename1, filename2)
	transform.tf_broadcast()

	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

