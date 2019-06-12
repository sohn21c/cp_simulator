#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import JointState


def two_arm():

    # init publisher node
    rospy.init_node('two_arm')
    pub = rospy.Publisher('joint_states', JointState, queue_size = 10)

    # set rate
    rate = rospy.Rate(50)

    # init JointState message
    joint = JointState()
    joint.header.stamp = rospy.Time.now()
    joint.name = ['base_to_upper','upper_to_elbow','elbow_to_lower']
    joint.position = [0,0,0]
    joint.velocity = []
    joint.effort = []

    # time
    t_0 = rospy.get_time()

    while not rospy.is_shutdown():
        # set t to zero when loop begins
        t_init = rospy.get_time()
        t = t_init - t_0

        # equation for x and y position w.r.t to time
        x = 0.5*np.cos(2*np.pi*t/5.0) + 1.25
        y = 0.5*np.sin(2*np.pi*t/5.0)

        # inverse kinematics for theta1 and theta2
        l1 = 1
        l2 = 1
        R = np.sqrt(x**2+y**2)

        alpha = np.arctan2(y,x)
        beta = np.arccos((R**2+l1**2-l2**2)/(2*R*l1))
        theta1 = alpha - beta
        gamma = np.arccos((l1**2+l2**2-R**2)/(2*l1*l2))
        theta2 = np.pi - gamma

        # publish joint angles
        joint.header.stamp = rospy.Time.now()
        joint.position = [theta1,theta2,0]
        pub.publish(joint)
        rate.sleep()


if __name__ == "__main__":
    try:
        two_arm()
    except rospy.ROSInterruptException:
        pass
