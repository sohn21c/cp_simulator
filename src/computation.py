#!/usr/bin/env python
"""
Author: James Sohn
Laast modified: 12/10/19

This module contains helper functions used for computation at each stage to find the position and rotation
"""
# import relavant libraries
import csv
import time
import math
import data_parser
import os
import numpy as np
import matplotlib.pyplot as plt 
import pandas as pd

# define constant
fps = 200

def R_2vect(vector_orig, vector_fin):
    """
    Calculate the rotation matrix required to rotate from one vector to another.
	
	input: 
		- vector_orig
		- vector_fin

	output:
		- 3x3 rotation matrix that rotate vector_orig to vector_fin
    """
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
    R = np.zeros([3,3])
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

	input:
		- w_x, w_y, w_z: constant angular velocity at give period w.r.t each 	axis in 3d

	output:
		- 3x3 rotation matrix from initial coordinate to final coordinate
	"""
	# define the array of w
	w = np.array([w_x, w_y, w_z])
	# scale up the w to avoid forcing norm to be zero
	w *= 1e6
	# if norm of the angular velocity is too small, return identity matrix
	if np.linalg.norm(w) < 1e-3:
		return np.identity(3)
	# find the fixed rotation axis from w
	v_x, v_y, v_z = w / np.linalg.norm(w)
	# scale back down the w
	w /= 1e6
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

def pos_rot_calculation(filename, sensor, start, end):
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
	# acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = data_parser.data_parser(filename, sensor, start, end)
	acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = data_parser.data_parser2(filename, sensor, start, end)
	# create the data index 
	ind = 0

	# initial gravity at t = 0
	# acc_x_init = acc_x[0]
	# acc_y_init = acc_y[0]
	# acc_z_init = acc_z[0]
	# initial acceleartion measurement, averaged from the first 50 measurements (.25sec)
	acc_x_init = np.average(acc_x[0:10])
	acc_y_init = np.average(acc_y[0:10])
	acc_z_init = np.average(acc_z[0:10])
	# # initial acceleartion measurement, max from the first 50 measurements (.25sec)
	# acc_x_init = np.max(acc_x[0:25])
	# acc_y_init = np.max(acc_y[0:25])
	# acc_z_init = np.max(acc_z[0:25])

	# compute gravity
	init_gravity = np.sqrt(acc_x_init**2 + acc_y_init**2 + acc_z_init**2)
	# gravity in z direction in world frame
	gravity_worldframe = np.array([0., 0., -init_gravity])
	# initial gravity in body frame
	init_gravity_bodyframe = np.array([acc_x_init, acc_y_init, acc_z_init])
	# find rotation matrix rotating bodyframe to world frame
	RR = R_2vect(init_gravity_bodyframe, gravity_worldframe)
	# initial frame
	x_abs = np.array([1., 0., 0.])
	y_abs = np.array([0., 1., 0.])
	z_abs = np.array([0., 0., 1.])
	# initial coordinate WF
	identity_coord_body = np.array([[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
	# find sensor coordinate in world frame
	sensor_coord_world = np.dot(RR, identity_coord_body)
	# define the initial velocity in world frame
	vel_world = np.array([0., 0., 0.])
	# define the initial position in world frame
	pos_world = np.array([0., 0., 0.])
	# containers for pose and orientation
	posx_time = []
	posy_time = []
	posz_time = []
	rot_time_00 = []
	rot_time_01 = []
	rot_time_02 = []
	rot_time_10 = []
	rot_time_11 = []
	rot_time_12 = []
	rot_time_20 = []
	rot_time_21 = []
	rot_time_22 = []
	acc_world_time = []
	acc_mag_time = []
	acc_mag_diff_time = []
	acc_mag_copy = 0.0
	# iterate through the measurement and update the position and orientation of the frame
	for i in range(len(acc_x)):
		# acceleration in array form at instant time step
		acc_body = np.array([acc_x[i], acc_y[i], acc_z[i]])
		# rotate the body frame acceleartion to world frame
		acc_world = np.dot(RR, acc_body)
		# subtract the gravitational acceleration. g in -z axis
		acc_world -= np.array([0, 0, -init_gravity])
		# integerate acceleartion to get velocity in world frame
		vel_world += acc_world * (1.0/fps)
		# integrate velocity to get position in wolrd frame
		pos_world += vel_world * (1.0/fps)
		# body frame rotation matrix from gyro measurement
		Rot_body = SORA(gyro_x[i], gyro_y[i], gyro_z[i])
		# find the next body frame with the Rot_body
		sensor_coord_world = np.dot(sensor_coord_world, Rot_body)
		# print statement for intermediate value check
		if i % 20 == 0:
			# print(acc_world)
			continue
		# update the Rotation matrix for world frame representation
		RR = np.dot(RR, Rot_body)
		# test the moving average and find stationary
		if i < len(acc_x) - 1:
			acc_mag_diff_time.append(np.linalg.norm(acc_world)-acc_mag_copy)
			acc_mag_copy = np.linalg.norm(acc_world)
		# append data point to container for plot
		acc_mag_time.append(np.linalg.norm(acc_world))
		acc_world_time.append(acc_world)
		posx_time.append(pos_world[0])
		posy_time.append(pos_world[1])
		posz_time.append(pos_world[2])
		rot_time_00.append(sensor_coord_world[0,0])
		rot_time_01.append(sensor_coord_world[0,1])
		rot_time_02.append(sensor_coord_world[0,2])
		rot_time_10.append(sensor_coord_world[1,0])
		rot_time_11.append(sensor_coord_world[1,1])
		rot_time_12.append(sensor_coord_world[1,2])
		rot_time_20.append(sensor_coord_world[2,0])
		rot_time_21.append(sensor_coord_world[2,1])
		rot_time_22.append(sensor_coord_world[2,2])

	# # position plot
	# plot_x = np.linspace(0, 100, len(posx_time))
	# plt.figure(3)
	# plt.plot(plot_x, posx_time, 'r', label='x')
	# plt.plot(plot_x, posy_time, 'g', label='y')
	# plt.plot(plot_x, posz_time, 'b', label='z')
	# plt.title("Position", loc='center')
	# plt.legend(loc='upper left')
	# plt.ylabel("[m]")
	# plt.xlabel("Relative time")
	# plt.show()

	# # acceleartion in world frame plot
	# plot_x = np.linspace(0, 100, len(acc_world_time))
	# plt.figure(4)
	# plt.plot(plot_x, acc_world_time)
	# plt.title("Linear acceleration in computation", loc='center')
	# # plt.legend(loc='upper left')
	# plt.ylabel("[m/s/s]")
	# plt.xlabel("Relative time")

	# # acceleartion magnitude plot
	# plot_x = np.linspace(0, 100, len(acc_world_time))
	# plt.figure(5)
	# plt.plot(plot_x, acc_mag_time)
	# plt.title("Linear acceleration magnitutde in computation", loc='center')
	# # plt.legend(loc='upper left')
	# plt.ylabel("[m/s/s]")
	# plt.xlabel("Relative time")
	# # plt.ylim(top=0.5, bottom=-0.5)
	# # plt.xlim(right=20, left=0)

	# # acceleartion magnitude diff plot: difference between previous and current time step
	# plot_x = np.linspace(0, 100, len(acc_world_time)-1)
	# plt.figure(6)
	# plt.plot(plot_x, acc_mag_diff_time)
	# plt.title("Linear acceleration magnitutde diff in computation", loc='center')
	# # plt.legend(loc='upper left')
	# plt.ylabel("[m/s/s]")
	# plt.xlabel("Relative time")
	# plt.ylim(top=0.5, bottom=-0.5)
	# plt.xlim(right=20, left=0)

	# acceleration magnitude moving average for 20 timesteps
	# acc_mag_average = []
	# for i in range(len(acc_mag_time)):
	# 	if i < 20:
	# 		acc_mag_average.append(acc_mag_time[i])
	# 	else:
	# 		acc_mag_average.append(np.sum(acc_mag_time[i-20:i])/20.0)

	# # acceleartion magnitude moving average plot
	# plot_x = np.linspace(0, 100, len(acc_world_time))
	# plt.figure(7)
	# plt.plot(plot_x, acc_mag_average)
	# plt.title("Linear acceleration magnitutde WITHOUT GRAVITY", loc='center')
	# # plt.legend(loc='upper left')
	# plt.ylabel("[m/s/s]")
	# plt.xlabel("Relative time")
	# # plt.ylim(top=0.5, bottom=-0.5)
	# # plt.xlim(right=20, left=0)

	# # peak difference(max - min) within 10 time steps
	# acc_peakdiff_time = []
	# for i in range(len(acc_mag_time)):
	# 	if acc_mag_average[i] > 2.0:
	# 		acc_peakdiff_time.append(2.0)
	# 	else:
	# 		if i == 0:
	# 			acc_peakdiff_time.append(acc_mag_average[i])
	# 		elif i < 10:
	# 			acc_peakdiff_time.append(np.max(acc_mag_average[0:i])-np.min(acc_mag_average[0:i]))
	# 		elif i >= 10:
	# 			acc_peakdiff_time.append(np.max(acc_mag_average[i-10:i])-np.min(acc_mag_average[i-10:i]))

	# acceleartion mag peak diff plot
	# plot_x = np.linspace(0, 100, len(acc_world_time))
	# plt.figure(8)
	# plt.plot(plot_x, acc_peakdiff_time)
	# plt.title("Linear acceleration magnitutde peak diff in computation", loc='center')
	# plt.legend(loc='upper left')
	# plt.ylabel("[m/s/s]")
	# plt.xlabel("Relative time")
	# plt.ylim(top=0.5, bottom=-0.5)
	# plt.xlim(right=20, left=0)

	# show plot
	# plt.show()

	return posx_time, posy_time, posz_time, rot_time_00, rot_time_01, rot_time_02, rot_time_10, rot_time_11, rot_time_12, rot_time_20,rot_time_21, rot_time_22

if __name__ == '__main__':
	# receive command line input for file name
	directory = input("> ")
	directory = '/home/james/Documents/final_project/James/data/' + directory
	filelist = os.listdir(directory)
	print('[INFO] Filelist: ',filelist)

	# time sync using more than two sets
	if len(filelist) == 4:
		ur = pd.read_csv(directory+'ur.tsv', sep='\t')
		ul = pd.read_csv(directory+'ul.tsv', sep='\t')
		ur_t = ur['local time']
		ul_t = ul['local time']
		r_start, r_end = data_parser.time_sync(ur_t, ul_t)
		l_start, l_end = 0, len(ul_t)
		print('[INFO] Master time sync ref points: ', r_start, r_end, r_end-r_start)
		print('[INFO] Slave time sync ref points: ',l_start, l_end, l_end-l_start)

	if len(filelist) >= 7:
		ur = pd.read_csv(directory+'ur.tsv', sep='\t')
		ul = pd.read_csv(directory+'ul.tsv', sep='\t')
		tr = pd.read_csv(directory+'tr.tsv', sep='\t')
		tl = pd.read_csv(directory+'tl.tsv', sep='\t')
		ur_t = ur['local time']
		ul_t = ul['local time']
		tr_t = tr['local time']
		tl_t = tl['local time']
		lLeg_start, lLeg_end = 0, len(tl_t)
		rLeg_start, rLeg_end = data_parser.time_sync(tr_t, tl_t)
		lArm_start, lArm_end = data_parser.time_sync(ul_t, tl_t)
		rArm_start, rArm_end = data_parser.time_sync(ur_t, tl_t)
		print('[INFO] Master time sync ref points: ', lLeg_start, lLeg_end, lLeg_end-lLeg_start)
		print('[INFO] Master time sync ref points: ', rLeg_start, rLeg_end, rLeg_end-rLeg_start)
		print('[INFO] Master time sync ref points: ', lArm_start, lArm_end, lArm_end-lArm_start)
		print('[INFO] Master time sync ref points: ', rArm_start, rArm_end, rArm_end-rArm_start)

	for file in filelist:
		try:
			if file in ['ur.tsv', 'lr.tsv']:
				start, end = rArm_start, rArm_end 
				if file == 'ur.tsv':
					sensor = '2'
				else:
					sensor = '1'
			elif file in ['ul.tsv', 'll.tsv']:
				start, end = lArm_start, lArm_end
				if file == 'ul.tsv':
					sensor = '4'
				else:
					sensor = '5'
			elif file in ['tr.tsv', 'sr.tsv']:
				start, end = rLeg_start, rLeg_end
				if file == 'tr.tsv':
					sensor = '6'
				else:
					sensor = '8'
			elif file in ['tl.tsv', 'sl.tsv']:
				start, end = lLeg_start, lLeg_end
				if file == 'tl.tsv':
					sensor = '9'
				else:
					sensor = '10'

		except NameError:
			print("[INFO] Two sensors or less used")
			df = pd.read_csv(directory+file, sep='\t')
			start = 0
			end = len(df)
			if file == 'ur.tsv':
				sensor = '1'
			else:
				sensor = '3'

		filename = directory + file
		simplified = filename	
		simplified = simplified.split('/')
		simplified = '_'.join(simplified[-3:])
		simplified = simplified.split('.')
		simplified = simplified[0]
		
		# publishes data from the file
		posx_time, posy_time, posz_time, \
		rot_time_00, rot_time_01, rot_time_02, \
		rot_time_10, rot_time_11, rot_time_12, \
		rot_time_20,rot_time_21, rot_time_22 = pos_rot_calculation(filename, sensor, start, end)

		# write the csv file
		writefile = '../demo/{}.csv'.format(simplified)
		with open(writefile, mode='w') as csvwriter:
			writer = csv.writer(csvwriter, delimiter = ',')
			for i in range(len(posx_time)):
				writer.writerow(
					[posx_time[i], posy_time[i], posz_time[i], 
					rot_time_00[i], rot_time_01[i], rot_time_02[i], 
					rot_time_10[i], rot_time_11[i], rot_time_12[i], 
					rot_time_20[i], rot_time_21[i], rot_time_22[i]])
	print("[INFO] Done")

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
