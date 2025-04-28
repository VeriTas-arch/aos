#!/usr/bin/python3

import cv2
import numpy as np
import rospy
import tf
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CompressedImage

from mathHelper import matrixHelper

bridge = CvBridge()


class ArucoLocater:
    def __init__(self):
        self.intrinsic = np.array(
            [[1.0458e3, 0, 935.1922], [0, 1.0457e3, 509.9960], [0, 0, 1]]
        )
        self.distortion = np.zeros((1, 5))
        self.image_path = "/image_raw/compressed"
        self.image_raw = None
        self.image_gray = None
        self.image_sub = rospy.Subscriber(
            self.image_path, CompressedImage, self.image_callback, queue_size=10
        )

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.rvec = None
        self.tvec = None
        self.alpha = None
        self.quat = None

    def image_callback(self, msg):
        self.image_raw = bridge.compressed_imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )
        self.image_gray = cv2.cvtColor(self.image_raw, cv2.COLOR_BGR2GRAY)
        self.update_aruco_location()

    def estimate_pose(self, corners, length):
        points_3d = (
            np.array(
                [[0, 0.5, -0.5], [0, 0.5, 0.5], [0, -0.5, 0.5], [0, -0.5, -0.5]],
                dtype=np.float32,
            )
            * length
        )
        points_2d = np.array(corners[0], dtype=np.float32)
        _, rvec, tvec = cv2.solvePnP(
            points_3d, points_2d, self.intrinsic, self.distortion
        )
        return rvec, tvec

    def update_aruco_location(self):
        if self.image_gray is None:
            return

        corners, ids, _ = self.detector.detectMarkers(self.image_gray)
        print("corners: ", corners)
        print("ids: ", ids)
        if ids is not None:
            self.corners = corners
            self.ids = ids
            if ids.size != 1:
                corners = corners[-1]
                ids = ids[-1]

            rvec, tvec = self.estimate_pose(corners, 0.15)
            print("pose:", rvec, tvec)

            R_camera_image = np.array(
                [[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]]
            )
            R_image_aruco = np.zeros((4, 4), dtype=np.float32)
            rot_mat = cv2.Rodrigues(rvec)
            R_image_aruco[:3, :3] = rot_mat[0]
            R_image_aruco[:3, 3] = tvec.flatten()
            R_image_aruco[3, 3] = 1

            R_camera_aruco = R_camera_image @ R_image_aruco

            self.quat = tf.transformations.quaternion_from_matrix(R_camera_aruco)
            self.tvec = R_camera_aruco[:3, 3].flatten()


class plane_part_with_agv:
    def __init__(self, nodename):
        rospy.init_node(nodename)
        self.xiangjiLink = "xiangji_Link"
        self.modulatingLink = "modulating_Link"
        self.platformLink = "platform_Link"
        self.bottomLink = "bottom_Link"
        self.referenceLink = "agv_part/reference_Link"
        self.imageTargetLink = "agv_part/image_target1"
        self.agvLink = "agv_part/agv_Link"

        self.bottomPlatformRel = np.array([0.0, 0.0, 0.614, 0.0, 0.0, 0.0, 1.0])

        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.aruco_locater = ArucoLocater()

    def lookup_transform_quaternion_vector(
        self, target_frame, source_frame, time, timeout
    ):
        trans_ = self.tfBuffer.lookup_transform(
            target_frame, source_frame, time, timeout
        )
        return np.array(
            [
                trans_.transform.translation.x,
                trans_.transform.translation.y,
                trans_.transform.translation.z,
                trans_.transform.rotation.x,
                trans_.transform.rotation.y,
                trans_.transform.rotation.z,
                trans_.transform.rotation.w,
            ]
        )

    def run(self):
        rate = rospy.Rate(10)
        trans = TransformStamped()

        imageTargetAgvRel = self.lookup_transform_quaternion_vector(
            self.imageTargetLink, self.agvLink, rospy.Time(0), rospy.Duration(1.0)
        )

        try:
            while not rospy.is_shutdown():
                # detection by aruco tags and gives quaternion from rotation vector and translation vector
                if self.aruco_locater.quat is not None:
                    xiangjiImageTargetRel = np.concatenate(
                        (self.aruco_locater.tvec, self.aruco_locater.quat), axis=0
                    )
                else:
                    xiangjiImageTargetRel = np.array([1.6, 0, 0, 0, 0, 0, 1])

                newTrans = matrixHelper.compose_quaternion_vector(
                    xiangjiImageTargetRel, imageTargetAgvRel
                )

                # publish the tf topic to make corrected pose of two robots
                trans.header.frame_id = self.xiangjiLink
                trans.child_frame_id = self.agvLink

                trans.header.stamp = rospy.Time.now()

                trans.transform.translation.x = newTrans[0]
                trans.transform.translation.y = newTrans[1]
                trans.transform.translation.z = newTrans[2]

                trans.transform.rotation.x = newTrans[3]
                trans.transform.rotation.y = newTrans[4]
                trans.transform.rotation.z = newTrans[5]
                trans.transform.rotation.w = newTrans[6]

                self.tfBroadcaster.sendTransform(trans)

                rate.sleep()

        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return


if __name__ == "__main__":
    service = plane_part_with_agv("digital_twining")

    service.run()
