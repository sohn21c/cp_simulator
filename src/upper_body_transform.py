#!/usr/bin/env python
"""
Author: James Sohn
Last Modified: 11/09/2019

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
from tf.transformations import euler_from_matrix, quaternion_from_euler, quaternion_multiply

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

		# default oritentation for sensors
		posQuat1 = (-0.707, 0., 0.707, 0.)
		posQuat2 = (0., 0., 0., 1.)
		posQuat3 = (-0.707, 0., 0.707, 0.)
		posQuat4 = (0., 0., 0., 1.)
		posQuat5 = (-0.707, 0., 0.707, 0.)
		posQuat6 = (0., 0., 0., 1.)
		posQuat7 = (-0.707, 0., 0.707, 0.)
		posQuat8 = (0., 0., 0., 1.)

		while not rospy.is_shutdown():
			# step increase the index and rewind when maxed
			if ind == len(self._posx1) - 1:
				ind = 0
				rospy.sleep(3)

			# convert angles to quaternion
			for i in range(1, self.num_sensors+1):
				exec('posRot = self._posRot%d' %i)
				euler_x, euler_y, euler_z = euler_from_matrix(posRot[ind])
				posQuat = quaternion_from_euler(euler_x, euler_y, euler_z) 	
				exec('posQuat%d = posQuat' %i)

			# convert rotational matrix relative to the world frame detached from the link in between
			if self.num_sensors > 1:
				quatinv1 = list(posQuat1)	# right arm
				quatinv1[3] *= -1.0
				posQuat2 = quaternion_multiply(quatinv1, posQuat2)

			if self.num_sensors > 3:
				quatinv3 = list(posQuat3)	# left arm
				quatinv3[3] *= -1.0
				posQuat4 = quaternion_multiply(quatinv3, posQuat4)
			
			if self.num_sensors > 5:
				quatinv5 = list(posQuat5)	# right leg
				quatinv5[3] *= -1.0
				posQuat6 = quaternion_multiply(quatinv5, posQuat6)

			if self.num_sensors > 7:
				quatinv7 = list(posQuat7)	# left leg
				quatinv7[3] *= -1.0
				posQuat8 = quaternion_multiply(quatinv7, posQuat8)

			# broadcast to tf
			# body
			self._br.sendTransform(
					(0., 0., 0.),	
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
					# (0., 0., 0., 1.),
					rospy.Time.now(), 
					"right_elbow", 
					"right_shoulder")
			# left arm
			self._br.sendTransform(
					(0., -0.15, 0.),	
					(posQuat3[0], posQuat3[1], posQuat3[2], posQuat3[3]), 
					# (0., 0., 0., 1.),
					rospy.Time.now(), 
					"left_shoulder", 
					"body")
			self._br.sendTransform(
					(0.25, 0., 0.),	
					(posQuat4[0], posQuat4[1], posQuat4[2], posQuat4[3]), 
					# (0., 0., 0., 1.),
					rospy.Time.now(), 
					"left_elbow", 
					"left_shoulder")
			# right leg
			self._br.sendTransform(
					(0., 0.075, -0.5),	
					(posQuat5[0], posQuat5[1], posQuat5[2], posQuat5[3]), 
					# (-0.707, 0., 0.707, 0.), 
					rospy.Time.now(), 
					"right_hip", 
					"body")
			self._br.sendTransform(
					(0.275, 0., 0.),	
					(posQuat6[0], posQuat6[1], posQuat6[2], posQuat6[3]), 
					# (0., 0., 0., 1.),
					rospy.Time.now(), 
					"right_knee", 
					"right_hip")
			# left leg
			self._br.sendTransform(
					(0., -0.075, -0.5),	
					(posQuat7[0], posQuat7[1], posQuat7[2], posQuat7[3]), 
					# (0., 0., 0., 1.),
					rospy.Time.now(), 
					"left_hip", 
					"body")
			self._br.sendTransform(
					(0.275, 0., 0.),	
					(posQuat8[0], posQuat8[1], posQuat8[2], posQuat8[3]), 
					# (-0.707, 0., 0.707, 0.), 
					rospy.Time.now(), 
					"left_knee", 
					"left_hip")

			# rospy.loginfo("broadcasting %f %f %f %f" %(posQuat1[0], posQuat1[1],posQuat1[2], posQuat1[3]))

			# increment the index for next time step
			ind += 1

			# take rate in effect
			self._rate.sleep()

# main
def main():
	# Demofile names go here
	filename1 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_ur.csv"
	filename2 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_lr.csv"
	filename3 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_ul.csv"
	filename4 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_ll.csv"
	# filename5 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_tr.csv"
	# filename6 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_sr.csv"
	# filename7 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_tl.csv"
	# filename8 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_sl.csv"
	
	# Successful demo
	# filename1 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_ur.csv"
	# filename2 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_lr.csv"
	# filename3 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_ul.csv"
	# filename4 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_ll.csv"
	# filename5 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_ur.csv"
	# filename6 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_lr.csv"
	# filename7 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_ul.csv"
	# filename8 = "/home/james/catkin_ws/src/cp_simulator/demo/" + "111019_run7_ll.csv"

	# instantiate the Transform class	
	# transform = Transform(num_sensors=8)
	# transform.get_pose(filename1, filename2, filename3, filename4, filename5, filename6, filename7, filename8)
	# transform.tf_broadcast()
	
	transform = Transform(num_sensors=4)
	transform.get_pose(filename1, filename2, filename3, filename4)
	transform.tf_broadcast()
	rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

