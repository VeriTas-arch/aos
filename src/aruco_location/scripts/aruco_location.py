#! /usr/bin/env python3

# Package Name: aruco_location

import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf

bridge=CvBridge()

class ArucoLocation:
    def __init__(self):
        self.intrinsic = np.array([[1.0458e3, 0, 935.1922], [0, 1.0457e3, 509.9960], [0, 0, 1]])
        # self.distortion = np.array([-0.0535, 0.2115, 0, 0, -0.1398])
        self.distortion = np.zeros((1,5))
        # self.intrinsic = np.array([[2270.613523237546, 0.0, 960.5], [0.0, 2270.613523237546, 540.5], [0.0, 0.0, 1.0]])
        # self.distortion = np.array([0, 0, 0, 0, 0])

        self.image_path = "/camera_image/compressed"
        # self.image_path = '/modulating_part/camera1/image_raw/compressed'
        self.image_raw = None
        self.image_gray = None
        self.image_sub = rospy.Subscriber(self.image_path, CompressedImage, self.image_callback, queue_size=10)

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.rvec = None
        self.tvec = None
        self.alpha = None
        self.quat = None

    def image_callback(self,msg):
        # print('Get Image')
        self.image_raw = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.image_gray = cv2.cvtColor(self.image_raw, cv2.COLOR_BGR2GRAY)
        # print(self.image_raw.shape)
        self.update_aruco_location()

    def estimate_pose(self,corners,length):
        points_3d = np.array([[0, 0.5, -0.5], [0, 0.5, 0.5], [0, -0.5, 0.5], [0, -0.5, -0.5]], dtype=np.float32) * length
        points_2d = np.array(corners[0], dtype=np.float32)
        _, rvec, tvec = cv2.solvePnP(points_3d, points_2d, self.intrinsic, self.distortion)
        return rvec, tvec

    def update_aruco_location(self):
        if self.image_gray is None:
            return
        # cv2.imshow("image",self.image_raw)
        # cv2.waitKey(0)
        corners, ids, _ = self.detector.detectMarkers(self.image_gray)
        # print("corners: ",corners)
        # print("ids: ",ids)
        if ids is not None:
            if ids.size != 1:
                corners = corners[0]
                ids = ids[0]
            rvec,tvec = self.estimate_pose(corners,0.03)

            R_camera_image = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
            R_image_aruco = np.zeros((4,4), dtype=np.float32)
            R_image_aruco[:3, :3] = cv2.Rodrigues(rvec)[0]
            R_image_aruco[:3, 3] = tvec.flatten()
            R_image_aruco[3, 3] = 1
            R_camera_aruco = R_camera_image @ R_image_aruco

            self.rvec = cv2.Rodrigues(R_camera_aruco[:3, :3])[0].flatten()
            self.tvec = R_camera_aruco[:3, 3]
            self.alpha = np.linalg.norm(self.rvec)
            norm_rvec = self.rvec / self.alpha
            self.quat = np.concatenate((norm_rvec * np.sin(self.alpha / 2), np.array([np.cos(self.alpha / 2)])), axis=0)

            # print("rvec: ", self.rvec)
            # print("tvec: ", self.tvec)
            # (r, p, y) = tf.transformations.euler_from_quaternion(self.quat)

if __name__=="__main__":
    rospy.init_node("aruco_location")
    rospy.loginfo("Aruco location node started")

    aruco_location=ArucoLocation()
    
    rate=rospy.Rate(1)
    cnt=0
    while not rospy.is_shutdown():
        cnt+=1
        print(f"Getting aruco location: {cnt}")
        print(f"rvec: {aruco_location.rvec}")
        print(f"quat: {aruco_location.quat}")
        rate.sleep()