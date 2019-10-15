"""
Date: 10/14/2019
Author: James Sohn

This is the module that imports the measurement data from the .tsv created by the sensor measurements and separates data into individual containers
"""
# import relevant libraries
import os
import csv
import numpy as np 
import matplotlib.pyplot as plt

def data_parser(filename):
	"""
	parses the measurements of accelerometer and gyroscope in 3 axes and separates them in each container

	input:
		- filename: name of the file in the same directory 
	output:
		- acc_x, acc_y, acc_z: acceleration for each axis in m/s/s
		- gyro_x, gyro_y, gyro_z: angular velocity w.r.t each axis in rad/s
	"""
	# get the CWD
	cwd = os.getcwd()

	# Define the data path for opening
	# datapath = cwd + "/" + filename
	datapath = filename

	# define the conversion coefficients
	acc_conv = 8 / 2**16 * 9.8065
	gyro_conv = 1000 / 2**16 * np.pi / 180

	# Open the txt file with the csv reader
	with open(datapath) as tsvfile:
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

		# Parse each line and store the data in each container
		for row in data[1:]:
			time_stamp.append(float(row[0]))
			# Acceleartion with conversion factor
			acc_x.append(float(row[1]) * acc_conv)	
			acc_y.append(float(row[2]) * acc_conv)
			acc_z.append(float(row[3]) * acc_conv)
			# # Angular velocity with conversion factor
			# gyro_x.append((float(row[4]) * gyro_conv))
			# gyro_y.append((float(row[5]) * gyro_conv))
			# gyro_z.append((float(row[6]) * gyro_conv))
			# sensor 2 unbias with offset
			gyro_x.append((float(row[4]) * gyro_conv) + 0.0016)
			gyro_y.append((float(row[5]) * gyro_conv) - 0.1197)
			gyro_z.append((float(row[6]) * gyro_conv) - 0.0075)
			# # sensor 3 unbias with offset
			# gyro_x.append((float(row[4]) * gyro_conv) - 0.0016)
			# gyro_y.append((float(row[5]) * gyro_conv) - 0.0101)
			# gyro_z.append((float(row[6]) * gyro_conv) + 0.0178)
			# # sensor M unbias with offset
			# gyro_x.append((float(row[4]) * gyro_conv) - 0.0313)
			# gyro_y.append((float(row[5]) * gyro_conv) + 0.0804)
			# gyro_z.append((float(row[6]) * gyro_conv) - 0.0527)
			# One can use this if sensor is not calibrated and biased
			# gyro_x.append((float(row[4]) - float(data[1][4])) * gyro_conv)
			# gyro_y.append((float(row[5]) - float(data[1][5])) * gyro_conv)
			# gyro_z.append((float(row[6]) - float(data[1][6])) * gyro_conv)

			# ##### test
			# input("hit enter")
			# print((float(row[5])))
			# print((float(row[5]) - float(data[1][5])) * gyro_conv)

	# (optional) print out average for grvity measurement
	# gravity = (np.average(acc_x), np.average(acc_y), np.average(acc_z))
	# print(np.average(np.linalg.norm(gravity)))

	# (optional) print out average for gyro measurement 
	# print(np.average(gyro_x))
	# print(np.average(gyro_y))
	# print(np.average(gyro_z))

	# (optional) plot accelearation for check
	plot_x = np.linspace(0, 100, len(acc_x))
	plt.figure(1)
	plt.plot(plot_x, acc_x, 'r', label='x')
	plt.plot(plot_x, acc_y, 'g', label='y')
	plt.plot(plot_x, acc_z, 'b', label='z')
	plt.title("Linear acceleration input in sensor frame", loc='center')
	plt.legend(loc='upper left')
	plt.ylabel("[m/s/s]")
	plt.xlabel("Relative time")
	# plt.show()

	# (optional) plot angle for check
	plot_x = np.linspace(0, 100, len(acc_x))
	plt.figure(2)
	plt.plot(plot_x, gyro_x, 'r', label='x')
	plt.plot(plot_x, gyro_y, 'g', label='y')
	plt.plot(plot_x, gyro_z, 'b', label='z')
	plt.title("Angular velocity input", loc='center')
	plt.legend(loc='upper left')
	plt.ylabel("[rad]")
	plt.xlabel("Relative time")
	plt.show()

	print (len(acc_x))

	return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z

# test
if __name__ == '__main__':
	filename = input('> ')
	acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = data_parser(filename)
	
