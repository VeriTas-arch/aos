#! /usr/bin/env python3

# Package Name: aruco_location

import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray

R03 = None
R01 = np.array([[1, 0, 0, 0.5], [0, 1, 0, 0], [0, 0, 1, 0.05], [0, 0, 0, 1]])
R32 = np.array([[1, 0, 0, -0.67], [0, 1, 0, 0], [0, 0, 1, -0.015], [0, 0, 0, 1]])


def callback(msg):
    # rospy.loginfo("Received data:\n"+str(np.array(msg.data).reshape((4,4))))
    R12 = np.array(msg.data).reshape((4, 4))
    R03 = np.dot(R01, np.dot(R12, np.linalg.inv(R32)))
    print(R03[:3, 3].flatten())


if __name__ == "__main__":
    rospy.init_node("aruco_listener")
    sub = rospy.Subscriber("/aruco_location", Float32MultiArray, callback, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # if R03 is not None:
        #     rospy.loginfo("R03:\n"+str(R03[:3,3].flatten()))
        rate.sleep()
