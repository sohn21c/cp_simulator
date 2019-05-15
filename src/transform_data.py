#!/usr/bin/env python

"""
Date: 05/09/2019
Author: James Sohn

This is the master node of the package to subscribe to the measurement data publishded by publish_data.py and publishes the Robot state and Joint State to Gazebo simulator
"""
# import relevant libraries
import rospy
import numpy as np 
import tf
import math
from cp_simulator.msg import Sensor
from sensor_msgs.msg import JointState

class Transform(object):
	def __init__(self):
		""" 
		initiate the parameters
		"""
		rospy.init_node('transform_data', anonymous=True)
		self._sensor_sub = rospy.Subscriber('sensor', Sensor, self.data_process)
		self._br = tf.TransformBroadcaster()
		self._rate = rospy.Rate(10.0)

		# conversion coefficients for data process
		self._fps = 200
		self._acc_conv = 8 / 2**16
		self._gyro_conv = 1000 / 2**16 * np.pi / 180
		return

	def data_process(self, data):
		"""
		process the data and returns
		input:
			data - data class Sensor that contains acceleration and gyroscope data

		output:

		"""
		# assign variable from the sensor data
		self._acc_x = data.acc_x
		self._acc_y = data.acc_y
		self._acc_z = data.acc_z
		self._gyro_x = data.gyro_x
		self._gyro_y = data.gyro_y
		self._gyro_z = data.gyro_z

		# loginfo to check receiving data
		rospy.loginfo("Receiving data %f, %f, %f, %f" %(self._acc_x, self._acc_y, self._acc_z, self._gyro_x))

		
	def tf_broadcast(self):
		# broadcasting to tf
		t = rospy.Time.now().to_sec() * math.pi
		self._br.sendTransform((2.0 * math.sin(t), 2.0 * math.cos(t), 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "world",
                         "chest")
		

# main
def main():
	transform = Transform()
	rate = rospy.Rate(10.0)

	# as main() is invoked in the script, while loop has to be in the main() to continuosly working
	while not rospy.is_shutdown():
		transform.tf_broadcast()
		# rospy.spin()	# rospy.spin() keeps python from exiting until the node is stopped
		rate.sleep()

	# # declare the subscriber node for sensor data
	# rospy.init_node('process_data', anonymous=True)

	# # initiate subscriber listening to topic sensor of type Sensor and calls data_process function
	# rospy.Subscriber('sensor', Sensor, data_process)

	# # set the communication rate to 200Hz compliant the sensor frequency
	# rate = rospy.Rate(1)

	# rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass

