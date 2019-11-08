"""
Author: James Sohn
Last modified: 11/08/2019

This is the module that imports the measurement data from the .tsv created by the sensor measurements and separates data into individual containers
"""
# import relevant libraries
import os
import csv
import numpy as np 
import matplotlib.pyplot as plt

def time_sync(time1, time2):
	"""
	finds the row index to time sync the collected data and returns start and end index
	"""
	ref = time2[1]
	ind = 0
	while abs(ref - time1[ind]) > 5:
		ind += 1
	start = time1[ind]

	return ind, ind+len(time2)

def data_parser(filename, sensor, start, end):
	"""
	parses the measurements of accelerometer and gyroscope in 3 axes and separates them in each container

	input:
		- filename: name of the file in the same directory 
		- start/end: time synced row index
	output:
		- acc_x, acc_y, acc_z: acceleration for each axis in m/s/s
		- gyro_x, gyro_y, gyro_z: angular velocity w.r.t each axis in rad/s
	"""
	# set the sensor bias
	x_bias, y_bias, z_bias = sensor_cfg(sensor)

	# define the conversion coefficients
	acc_conv = 8.0 / 2**16 * 9.8065
	gyro_conv = 1000.0 / 2**16 * np.pi / 180.0

	# Open the txt file with the csv reader
	with open(filename) as tsvfile:
		# Define csv.reader class for parsing
		reader = csv.reader(tsvfile, delimiter = '\t')

		# List-ify the incoming file
		data = list(reader)

		# Store the header file
		header = data[0]

		# Initiate the container for each data point
		time_stamp = []
		acc_x = []
		acc_y = []
		acc_z = []
		gyro_x = []
		gyro_y = []
		gyro_z = []
		ind = 0

		# Parse each line and store the data in each container
		for row in data[1:]:
			if ind >= start and ind < end:
				time_stamp.append(float(row[0]))

				# Acceleartion with conversion factor
				acc_x.append(float(row[1]) * acc_conv)	
				acc_y.append(float(row[2]) * acc_conv)
				acc_z.append(float(row[3]) * acc_conv)
				
				# # Angular velocity with conversion factor
				# gyro_x.append((float(row[4]) * gyro_conv))
				# gyro_y.append((float(row[5]) * gyro_conv))
				# gyro_z.append((float(row[6]) * gyro_conv))

				# Angular velocity with conversion factor
				# gyro_x.append(((float(row[4]) - float(x_bias)) * gyro_conv))
				# gyro_y.append(((float(row[5]) - float(y_bias)) * gyro_conv))
				# gyro_z.append(((float(row[6]) - float(z_bias)) * gyro_conv))

				# # One can use this if sensor is not calibrated and biased
				gyro_x.append((float(row[4]) - float(data[1][4])) * gyro_conv)
				gyro_y.append((float(row[5]) - float(data[1][5])) * gyro_conv)
				gyro_z.append((float(row[6]) - float(data[1][6])) * gyro_conv)

			ind += 1

	# (optional) print out average for grvity measurement
	# gravity = (np.average(acc_x), np.average(acc_y), np.average(acc_z))
	# print(np.average(np.linalg.norm(gravity)))

	# (optional) print out average for gyro measurement 
	# print(np.average(gyro_x))
	# print(np.average(gyro_y))
	# print(np.average(gyro_z))

	# (optional) plot accelearation for check
	# plot_x = np.linspace(0, 100, len(acc_x))
	# plt.figure(1)
	# plt.plot(plot_x, acc_x, 'r', label='x')
	# plt.plot(plot_x, acc_y, 'g', label='y')
	# plt.plot(plot_x, acc_z, 'b', label='z')
	# plt.title("Linear acceleration input in sensor frame", loc='center')
	# plt.legend(loc='upper left')
	# plt.ylabel("[m/s/s]")
	# plt.xlabel("Relative time")
	# plt.show()

	# (optional) plot angular velocity for check
	time = range(end-start)
	plt.figure(2)
	plt.plot(time, gyro_x, 'r', label='x')
	plt.plot(time, gyro_y, 'g', label='y')
	plt.plot(time, gyro_z, 'b', label='z')
	plt.title("Angular velocity input", loc='center')
	plt.legend(loc='upper left')
	plt.ylabel("[rad]")
	plt.xlabel("Relative time")
	plt.show()

	return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z


def sensor_cfg(sensor):
	"""
	parses sensor configuration file and select right bias for sensor

	input:
		sensor: sensor number in string\

	returns:
		sensor_bias for each axis
	"""

	sensor_cfg = '/home/james/catkin_ws/src/cp_simulator/cfg/sensor.cfg'
	f = open(sensor_cfg, 'r')
	contents = f.read()
	contents = contents.split('\n')

	sensor_bias = {}
	for line in contents:
		items = line.split(' ')
		if len(items) <= 1:
			continue
		
		sensor_bias[items[0]] = {}
		sensor_bias[items[0]]['x'] = items[1]
		sensor_bias[items[0]]['y'] = items[2]
		sensor_bias[items[0]]['z'] = items[3]

	x_bias = sensor_bias[sensor]['x']
	y_bias = sensor_bias[sensor]['y']
	z_bias = sensor_bias[sensor]['z']

	return x_bias, y_bias, z_bias


# test
if __name__ == '__main__':
	# file = input('name of the file: > ')
	# acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = data_parser(file)

	sensor_cfg()
