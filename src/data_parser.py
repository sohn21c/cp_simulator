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
import pandas as pd
from scipy.signal import butter, lfilter

def butter_lowpass(highcut, fs, order=5):
    nyq = 0.5 * fs
    high = highcut / nyq
    b, a = butter(order, high, btype='low')
    return b, a

def butter_lowpass_filter(data, highcut, fs, order=5):
    b, a = butter_lowpass(highcut, fs, order=order)
    y = lfilter(b, a, data)
    return y	

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
	
def smooth(x, window_len=11, window='hanning'):
	"""
	smoothes the data using and moving window

	window_len: has to be odd integer
	"""
	print('[INFO] original length: ', len(x))
	s = np.r_[x[window_len-1:0:-1],x,x[-2:-window_len-1:-1]]
	if window == 'flat':
		w = np.ones(window_len,'d')
	else:
		w = eval('np.'+window+'(window_len)')
	y = np.convolve(w/w.sum(), s, mode='valid')
	print('[INFO] processed length: ', len(y))
	
	return y

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

def data_parser2(filename, sensor, start, end):
	# define the conversion coefficients
	acc_conv = 8.0 / 2**16 * 9.8065
	gyro_conv = 1000.0 / 2**16 * np.pi / 180.0
	# read dataframe
	df = pd.read_csv(filename, sep='\t') 
	time_stamp = df['local time']
	acc_x = df['accel x']
	acc_y = df['accel y']
	acc_z = df['accel z']
	gyro_x = df['gyro x']
	gyro_y = df['gyro y']
	gyro_z = df['gyro z']
	# unbias the data
	gyro_x -= gyro_x[1000]
	gyro_y -= gyro_y[1000]
	gyro_z -= gyro_z[1000]
	# process data points
	lc = 0.0001
	hc = 10.
	fs = 200.
	order = 3
	gyro_x = butter_lowpass_filter(gyro_x, hc, fs, order)
	gyro_y = butter_lowpass_filter(gyro_y, hc, fs, order)
	gyro_z = butter_lowpass_filter(gyro_z, hc, fs, order)
	acc_x *= acc_conv
	acc_y *= acc_conv
	acc_z *= acc_conv
	gyro_x *= gyro_conv 
	gyro_y *= gyro_conv
	gyro_z *= gyro_conv
	acc_x = acc_x[start:end].reset_index()['accel x']
	acc_y = acc_y[start:end].reset_index()['accel y']
	acc_z = acc_z[start:end].reset_index()['accel z']
	gyro_x = gyro_x[start:end]
	gyro_y = gyro_y[start:end]
	gyro_z = gyro_z[start:end]

	# (optional) plot gyro data
	plt.figure(1)
	time = range(len(gyro_x))
	plt.plot(time, gyro_x, 'r', label='x')
	plt.plot(time, gyro_y, 'g', label='y')
	plt.plot(time, gyro_z, 'b', label='z')
	plt.title(f'Sensor {sensor} Gyro vs Time')
	plt.show()

	return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z

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
				# Angular velocity minus bias values with conversion factor
				# gyro_x.append(((float(row[4]) - float(x_bias)) * gyro_conv))
				# gyro_y.append(((float(row[5]) - float(y_bias)) * gyro_conv))
				# gyro_z.append(((float(row[6]) - float(z_bias)) * gyro_conv))

				# # One can use this if sensor is not calibrated and biased
				gyro_x.append((float(row[4]) - float(data[1][4])) * gyro_conv)
				gyro_y.append((float(row[5]) - float(data[1][5])) * gyro_conv)
				gyro_z.append((float(row[6]) - float(data[1][6])) * gyro_conv)
			ind += 1

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

# test
if __name__ == '__main__':
	file = input('name of the file: > ')
	acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = data_parser2(file, 0, 0, 0)

	# sensor_cfg()
