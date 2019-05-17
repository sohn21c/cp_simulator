"""
Date: 04/29/2019
Author: James Sohn

This is the module that imports the measurement data from the .tsv created by the sensor measurements and separates data separate containers
"""
# import relevant libraries
import os
import csv
import numpy as np 
import matplotlib.pyplot as plt

def data_parser(filename):
	# get the CWD
	cwd = os.getcwd()

	# Define the data path for opening
	datapath = cwd + "/" + filename

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
			acc_x.append(float(row[1]) * acc_conv)
			acc_y.append(float(row[2]) * acc_conv)
			acc_z.append(float(row[3]) * acc_conv)
			# un-bias the gyroscope data
			gyro_x.append((float(row[4]) - float(data[1][4])) * gyro_conv)
			gyro_y.append((float(row[5]) - float(data[1][5])) * gyro_conv)
			gyro_z.append((float(row[6]) - float(data[1][6])) * gyro_conv)

	# # plot for check
	# plot_x = np.linspace(0, 100, len(acc_x))
	# plt.plot(plot_x, acc_x)
	# plt.plot(plot_x, acc_y)
	# plt.plot(plot_x, acc_z)
	# plt.show()

	# # plot for check
	# plot_x = np.linspace(0, 100, len(acc_x))
	# plt.plot(plot_x, gyro_x)
	# plt.plot(plot_x, gyro_y)
	# plt.plot(plot_x, gyro_z)
	# plt.show()

	return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z


# test
if __name__ == '__main__':
	filename = input('> ')
	acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = data_parser(filename)
	
