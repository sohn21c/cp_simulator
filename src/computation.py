#!/usr/bin/env python
"""
Date: 05/13/2019
Author: James Sohn

This module contains helper functions used for computation at each stage to find the position and rotation
"""
# import relavant libraries
import csv
import rospy
import time
import math
import data_parser
import numpy as np
import matplotlib.pyplot as plt 
# from tf.transformations import quaternion_from_euler

# define constant
fps = 200

def R_2vect(vector_orig, vector_fin):
    """Calculate the rotation matrix required to rotate from one vector to another.

    For the rotation of one vector to another, there are an infinit series of rotation matrices possible.  Due to axially symmetry, the rotation axis can be any vector lying in the symmetry
    plane between the two vectors.  Hence the axis-angle convention will be used to construct the
    matrix with the rotation axis defined as the cross product of the two vectors.  The rotation
    angle is the arccosine of the dot product of the two unit vectors.

    Given a unit vector parallel to the rotation axis, w = [x, y, z] and the rotation angle a,
    the rotation matrix R is::

              |  1 + (1-cos(a))*(x*x-1)   -z*sin(a)+(1-cos(a))*x*y   y*sin(a)+(1-cos(a))*x*z |
        R  =  |  z*sin(a)+(1-cos(a))*x*y   1 + (1-cos(a))*(y*y-1)   -x*sin(a)+(1-cos(a))*y*z |
              | -y*sin(a)+(1-cos(a))*x*z   x*sin(a)+(1-cos(a))*y*z   1 + (1-cos(a))*(z*z-1)  |


    @param R:           The 3x3 rotation matrix to update.
    @type R:            3x3 numpy array
    @param vector_orig: The unrotated vector defined in the reference frame.
    @type vector_orig:  numpy array, len 3
    @param vector_fin:  The rotated vector defined in the reference frame.
    @type vector_fin:   numpy array, len 3
    """

    # Create empty rotation matrix
    R = np.zeros([3,3])

    # Convert the vectors to unit vectors.
    vector_orig = vector_orig / np.linalg.norm(vector_orig)
    vector_fin = vector_fin / np.linalg.norm(vector_fin)

    # The rotation axis (normalised).
    axis = np.cross(vector_orig, vector_fin)
    axis_len = np.linalg.norm(axis)
    if axis_len != 0.0:
        axis = axis / axis_len

    # Alias the axis coordinates.
    x = axis[0]
    y = axis[1]
    z = axis[2]

    # The rotation angle.
    angle = math.acos(np.dot(vector_orig, vector_fin))

    # Trig functions (only need to do this maths once!).
    ca = np.cos(angle)
    sa = np.sin(angle)

    # Calculate the rotation matrix elements.
    R[0,0] = 1.0 + (1.0 - ca)*(x**2 - 1.0)
    R[0,1] = -z*sa + (1.0 - ca)*x*y
    R[0,2] = y*sa + (1.0 - ca)*x*z
    R[1,0] = z*sa+(1.0 - ca)*x*y
    R[1,1] = 1.0 + (1.0 - ca)*(y**2 - 1.0)
    R[1,2] = -x*sa+(1.0 - ca)*y*z
    R[2,0] = -y*sa+(1.0 - ca)*x*z
    R[2,1] = x*sa+(1.0 - ca)*y*z
    R[2,2] = 1.0 + (1.0 - ca)*(z**2 - 1.0)

    return R

def SORA(w_x, w_y, w_z):
	"""
	Simultaneous Orthogonal Rotation Angle(SORA)
	finds the rotation matrix at fixed axis with angle from orthogonal angular velocity measurement

	Reference: https://www.hindawi.com/journals/js/2018/9684326/
	"""
	# define the array of w
	w = np.array([w_x, w_y, w_z])

	# if norm of the angular velocity is too small, return identity matrix
	if np.linalg.norm(w) < 1e-3:
		return np.identity(3)

	# find the fixed rotation axis from w
	v_x, v_y, v_z = w / np.linalg.norm(w)

	# calculate the amount of rotation psi, norm of angluar velocity multiplied by the period of measurement
	psi = np.linalg.norm(w) * (1.0/fps)

	# compute the elements
	R = np.zeros([3,3])
	R[0,0] = np.cos(psi) + v_x**2 * (1-np.cos(psi))
	R[0,1] = v_x * v_y * (1-np.cos(psi)) - v_z * np.sin(psi)
	R[0,2] = v_x * v_z * (1-np.cos(psi)) + v_y * np.sin(psi)
	R[1,0] = v_x * v_y * (1-np.cos(psi)) + v_z * np.sin(psi)
	R[1,1] = np.cos(psi) + v_y**2 * (1-np.cos(psi))
	R[1,2] = v_y * v_z * (1-np.cos(psi)) - v_x * np.sin(psi)
	R[2,0] = v_x * v_z * (1-np.cos(psi)) - v_y * np.sin(psi)
	R[2,1] = v_y * v_z * (1-np.cos(psi)) + v_x * np.sin(psi)
	R[2,2] = np.cos(psi) + v_z**2 * (1-np.cos(psi))

	return R

# rotation matrix of vector at fixed axis with angle
def rotation_mat(u, w):
	"""
	- Matrix logarithm
	finds the rotation matrix at fixed axis of u with angle w
	"""
	# extract vector in each axis from u
	u_x = u[0]
	u_y = u[1]
	u_z = u[2]

	# create empty container
	Rot = np.zeros([3,3])

	# compute the elements
	Rot[0,0] = np.cos(w) + u_x**2 * (1-np.cos(w))
	Rot[0,1] = u_x * u_y * (1-np.cos(w)) - u_z * np.sin(w)
	Rot[0,2] = u_x * u_z * (1-np.cos(w)) + u_y * np.sin(w)
	Rot[1,0] = u_x * u_y * (1-np.cos(w)) + u_z * np.sin(w)
	Rot[1,1] = np.cos(w) + u_y**2 * (1-np.cos(w))
	Rot[1,2] = u_y * u_z * (1-np.cos(w)) - u_x * np.sin(w)
	Rot[2,0] = u_x * u_z * (1-np.cos(w)) - u_y * np.sin(w)
	Rot[2,1] = u_y * u_z * (1-np.cos(w)) + u_x * np.sin(w)
	Rot[2,2] = np.cos(w) + u_z**2 * (1-np.cos(w))

	return Rot

# transformation matrix
def transformation_mat(a, b, c, A, B, C):
	"""
	finds transformation matrix between two cartesian coordinate
	"""
	Trans = np.zeros([3,3])
	Trans[0,0] = np.dot(a, A)
	Trans[0,1] = np.dot(a, B)
	Trans[0,2] = np.dot(a, C)
	Trans[1,0] = np.dot(b, A)
	Trans[1,1] = np.dot(b, B)
	Trans[1,2] = np.dot(b, C)
	Trans[2,0] = np.dot(c, A)
	Trans[2,1] = np.dot(c, B)
	Trans[2,2] = np.dot(c, C)

	return Trans

##### test module for pose_only
def pose_only(filename):
	"""
	test module to integrate the acceleartion only to test the behaviour
	"""
	# retrieve the data from the data_parser module
	acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = data_parser.data_parser(filename)

	# create the data index 
	ind = 0

	# initial gravity at t = 0
	acc_x_init = acc_x[0]
	acc_y_init = acc_y[0]
	acc_z_init = acc_z[0]

	# compute gravity
	init_gravity = np.sqrt(acc_x_init**2 + acc_y_init**2 + acc_z_init**2)

	# gravity in z direction in world frame
	gravity_worldframe = np.array([0, 0, init_gravity])

	# subtract the gravity from the acceleration in body frame
	for i in range(len(acc_x)):
		acc_x[i] -= acc_x_init
		acc_y[i] -= acc_y_init
		acc_z[i] -= acc_z_init

	# plot for check
	plot_x = np.linspace(0, 100, len(acc_x))
	plt.plot(plot_x, acc_x, 'r')
	plt.plot(plot_x, acc_y, 'g')
	plt.plot(plot_x, acc_z, 'b')
	plt.title("acceleartion")
	plt.show()

	# integrate acceleration for velocity
	vel_x = np.cumsum(acc_x, axis = 0)
	vel_y = np.cumsum(acc_y, axis = 0)
	vel_z = np.cumsum(acc_z, axis = 0)

	# plot for check
	plot_x = np.linspace(0, 100, len(acc_x))
	plt.plot(plot_x, vel_x, 'r')
	plt.plot(plot_x, vel_y, 'g')
	plt.plot(plot_x, vel_z, 'b')
	plt.title("velocity")
	plt.show()

	# integrate velocity for position
	pos_x = np.cumsum(vel_x, axis = 0)
	pos_y = np.cumsum(vel_y, axis = 0)
	pos_z = np.cumsum(vel_z, axis = 0)
	
	# plot for check
	plot_x = np.linspace(0, 100, len(acc_x))
	plt.plot(plot_x, pos_x, 'r')
	plt.plot(plot_x, pos_y, 'g')
	plt.plot(plot_x, pos_z, 'b')
	plt.title("position")
	plt.show()

	return pos_x, pos_y, pos_z

def pos_rot_calculation(filename):
	"""
	computes the position and rotation based on the measurement

	input:
		filename - name of the .tsv file
	output:
		pos_time - double integrated position from acceleration
		x_time - axis vector at each time step
		y_time - axis vector at each time step
		z_time - axis vector at each time step
	"""
	# retrieve the data from the data_parser module
	acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = data_parser.data_parser(filename)

	# create the data index 
	ind = 0

	# initial gravity at t = 0
	acc_x_init = acc_x[0]
	acc_y_init = acc_y[0]
	acc_z_init = acc_z[0]

	# initial acceleartion measurement, averaged from the first 50 measurements (.25sec)
	# acc_x_init = np.average(acc_x[0:10])
	# acc_y_init = np.average(acc_y[0:10])
	# acc_z_init = np.average(acc_z[0:10])

	# # initial acceleartion measurement, max from the first 50 measurements (.25sec)
	# acc_x_init = np.max(acc_x[0:25])
	# acc_y_init = np.max(acc_y[0:25])
	# acc_z_init = np.max(acc_z[0:25])

	# compute gravity
	init_gravity = np.sqrt(acc_x_init**2 + acc_y_init**2 + acc_z_init**2)

	# gravity in z direction in world frame
	gravity_worldframe = np.array([0, 0, -init_gravity])

	# initial gravity in body frame
	init_gravity_bodyframe = np.array([acc_x_init, acc_y_init, acc_z_init])

	# find rotation matrix rotating bodyframe to world frame
	RR = R_2vect(init_gravity_bodyframe, gravity_worldframe)

	# world frame vectors
	x_abs = np.array([1, 0, 0])
	y_abs = np.array([0, 1, 0])
	z_abs = np.array([0, 0, 1])
	coord_unit = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

	# find sensor coordinate in world frame
	coord_sensor = np.dot(RR, coord_unit)

	# define the initial velocity in world frame
	vel_world = np.array([0., 0., 0.])

	# define the initial position in world frame
	pos_world = np.array([0., 0., 0.])

	# containers for pose and orientation
	posx_time = []
	posy_time = []
	posz_time = []

	# iterate through the measurement and update the position and orientation of the frame
	for i in range(len(acc_x)):
		# acceleration in array form at instant time step
		acc_body = np.array([acc_x[i], acc_y[i], acc_z[i]])

		# rotate the body frame acceleartion to world frame
		acc_world = np.dot(RR, acc_body)
		
		# subtract the gravitational acceleration
		acc_world -= np.array([0, 0, -init_gravity])

		# integerate acceleartion to get velocity in world frame
		vel_world += acc_world * (1.0/fps)

		# integrate velocity to get position in wolrd frame
		pos_world += vel_world * (1.0/fps)

		# body frame rotation matrix from gyro measurement
		Rot_body = SORA(gyro_x[i], gyro_y[i], gyro_z[i])

		# find the next body frame with the Rot_body
		coord_sensor = np.dot(coord_sensor, Rot_body)

		# update the Rotation matrix for world frame representation
		# RR = np.dot(RR, Rot_body)

		# append to container
		posx_time.append(pos_world[0])
		posy_time.append(pos_world[1])
		posz_time.append(pos_world[2])

	# plot for check
	# plot_x = np.linspace(0, 100, len(posx_plot))
	# plt.plot(plot_x, posx_plot, 'r')
	# plt.plot(plot_x, posy_plot, 'g')
	# plt.plot(plot_x, posz_plot, 'b')
	# plt.show()

	return posx_time, posy_time, posz_time

if __name__ == '__main__':
	try:
		# receive command line input for file name
		filename = input("> ")
		
		# publishes data from the file
		posx_time, posy_time, posz_time = pos_rot_calculation(filename)

		# write the csv file
		with open('pose.csv', mode='w') as csvwriter:
			writer = csv.writer(csvwriter, delimiter = ',')
			for i in range(len(posx_time)):
				writer.writerow([posx_time[i], posy_time[i], posz_time[i]])

		#### test fn.vector rotation (done)
		# a_body = np.array([1., 0., 0.])
		# a_world = np.array([1./np.sqrt(2), 1./np.sqrt(2), 0.])
		# RR = R_2vect(a_body, a_world)
		# print(RR)
		# print(np.dot(RR.T, a_world))

		#### test pose only(no transformation)
		# pos_x, pos_y, pos_z = pose_only(filename)
		# #### write the csv file
		# with open('pose.csv', mode='w') as csvwriter:
		# 	writer = csv.writer(csvwriter, delimiter = ',')

		# 	for i in range(len(pos_x)):
		# 		writer.writerow([pos_x[i], pos_y[i], pos_z[i]])

		##### test for rotation matrix
		# a = np.array([1., 0., 0.])
		# b = np.array([0., 1., 0.])
		# rotation = rotation_mat(a, b)
		# print (rotation)
		# print(np.matmul(rotation, a))
		

	except rospy.ROSInterruptException:
		pass
