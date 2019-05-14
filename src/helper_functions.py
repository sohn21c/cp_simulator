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
import numpy as np

# rotation matrix
def rotation_mat(a, b):
	"""
	finds the 3x3 rotation matrix that rotates vector 'a' to vector 'b'
	b = Rot * a
	"""
	A = a / np.linalg.norm(a)
	B = b / np.linalg.norm(b)
	# skey symmetric matrix
	skew_sym = lambda x: np.array([[0, -x[2], x[1]],
									[x[2], 0, -x[0]],
									[-x[1], x[0], 0]])
	# Rodrigues' formula
	Rot = np.identity(3) + skew_sym(np.cross(A, B)) + \
			np.matmul(skew_sym(np.cross(A, B)),skew_sym(np.cross(A, B))) * (1-np.dot(A, B))

	return Rot

# define data_publisher
def data_pub(filename):
	# retrieve the data from the data_parser module
	acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = data_parser.data_parser(filename)
	# create the data index 
	ind = 0
	
	# initial acceleartion measurement
	init_gravity = np.square(acc_x[ind]**2 + acc_y[ind]**2 + acc_z[ind]**2)
	# gravity in z direction
	gravity_worldframe = np.array([0, 0, init_gravity])
	# find initial rotation matrix of sensor


if __name__ == '__main__':
	try:
		# # receive command line input for file name
		# filename = input("> ")
		# # publishes data from the file
		# data_pub(filename)
		a = np.array([1., 0., 0.])
		b = np.array([0., 1., 0.])
		rotation = rotation_mat(a, b)
		print (rotation)
		print(np.matmul(rotation, a))
		

	except rospy.ROSInterruptException:
		pass
