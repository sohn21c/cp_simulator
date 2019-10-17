#!/usr/bin/env python
"""
Date: 10/15/2019
Author: James Sohn

This is a node script to read position and angle data from .csv files and convert them to human-like geometry in Rviz
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
	def __init__(self, num_sensors):
		""" 
		initiate the parameters pertinent to the number of sensors used for simulation
		"""
		rospy.init_node('two_arm_transform', anonymous=True)
		self.num_sensors = num_sensors
		self._br = tf.TransformBroadcaster()
		self._rate = rospy.Rate(200.0)

		# create emptpy containers for each data points
		for i in range(1, num_sensors+1):
			exec('self._posx%d = []' %i)
			exec('self._posy%d = []' %i)
			exec('self._posz%d = []' %i)
			exec('self._posRot%d = []' %i)

		return

	def get_pose(self, *files):
		"""
		reads the pose from the .csv file
		"""
		# Assert the right number of files provided
		assert len(files) == self.num_sensors, 'Wrong number of .csv files provided'

		# Assign data points to right container
		for index, file in enumerate(files, 1):
			with open(file, mode='r') as csvreader:
				csv_reader = csv.reader(csvreader, delimiter=',')
				pos_time = list(csv_reader)				
				exec('pos_x = self._posx%d' %index)
				exec('pos_y = self._posy%d' %index)
				exec('pos_z = self._posz%d' %index)
				exec('posRot = self._posRot%d' %index)
				for i in range(len(pos_time)):
					pos_x.append(float(pos_time[i][0]))
					pos_y.append(float(pos_time[i][1]))
					pos_z.append(float(pos_time[i][2]))
					posRot.append(np.array([[float(pos_time[i][3]), float(pos_time[i][4]), float(pos_time[i][5])], [float(pos_time[i][6]), float(pos_time[i][7]), float(pos_time[i][8])], [float(pos_time[i][9]), float(pos_time[i][10]), float(pos_time[i][11])]]))

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
			for i in range(1, self.num_sensors+1):
				exec('posRot = self._posRot%d' %i)
				euler_x, euler_y, euler_z = euler_from_matrix(posRot[ind])
				posQuat = quaternion_from_euler(euler_x, euler_y, euler_z) 	
				exec('posQuat%d = posQuat' %i)

			# broadcast to tf
			# RVIZ axis flipped to real axis. All axis multiplied by (-1)
			# body
			self._br.sendTransform(
					(0., 0., 0.),	
					# (posQuat2[0], posQuat2[1], posQuat2[2], posQuat2[3]), 
					(0., 0., 0., 1.),
					rospy.Time.now(), 
					"body", 
					"base_link")
			# right arm
			self._br.sendTransform(
					(0., 0.15, 0.),	
					(posQuat1[0], posQuat1[1], posQuat1[2], posQuat1[3]), 
					# (0., 0., 0., 1.), 
					rospy.Time.now(), 
					"right_shoulder", 
					"body")
			self._br.sendTransform(
					(0.25, 0., 0.),	
					(posQuat2[0], posQuat2[1], posQuat2[2], posQuat2[3]), 
					rospy.Time.now(), 
					"right_elbow", 
					"right_shoulder")
			# left arm
			self._br.sendTransform(
					(0., -0.15, 0.),	
					(posQuat3[0], posQuat3[1], posQuat3[2], posQuat3[3]), 
					rospy.Time.now(), 
					"left_shoulder", 
					"body")
			self._br.sendTransform(
					(0.25, 0., 0.),	
					(posQuat4[0], posQuat4[1], posQuat4[2], posQuat4[3]), 
					rospy.Time.now(), 
					"left_elbow", 
					"left_shoulder")
			# right leg
			self._br.sendTransform(
					(0., 0.075, -0.5),	
					# (posQuat5[0], posQuat5[1], posQuat5[2], posQuat5[3]), 
					(-0.707, 0., 0.707, 0.), 
					rospy.Time.now(), 
					"right_hip", 
					"body")
			self._br.sendTransform(
					(0.275, 0., 0.),	
					# (posQuat6[0], posQuat6[1], posQuat6[2], posQuat6[3]), 
					(0., 0., 0., 1.),
					rospy.Time.now(), 
					"right_knee", 
					"right_hip")
			# left leg
			self._br.sendTransform(
					(0., -0.075, -0.5),	
					# (posQuat7[0], posQuat7[1], posQuat7[2], posQuat7[3]), 
					(0., 0., 0., 1.),
					rospy.Time.now(), 
					"left_hip", 
					"body")
			self._br.sendTransform(
					(0.275, 0., 0.),	
					# (posQuat8[0], posQuat8[1], posQuat8[2], posQuat8[3]), 
					(-0.707, 0., 0.707, 0.), 
					rospy.Time.now(), 
					"left_knee", 
					"left_hip")

			# rospy.loginfo("broadcasting %f %f %f %f" %(posQuat[0], posQuat[1],posQuat[2], posQuat[3]))

			# increment the index for next time step
			ind += 1

			# take rate in effect
			self._rate.sleep()

# main
def main():
	# instantiate the Transform class	
	# filename = raw_input("filename > ")
	filename1 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "demo10_upper.csv"
	filename2 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "demo10_fore.csv"
	transform = Transform(8)
	transform.get_pose(filename1, filename2, filename1, filename2, filename1, filename2, filename1, filename2)
	transform.tf_broadcast()

	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

