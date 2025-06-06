#!/usr/bin/python3

# import roslib
import cv2
import numpy as np
import rospy
import tf
import tf2_ros
from config import Config
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, TransformStamped, Twist
from mathHelper import matrixHelper
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float64MultiArray
from tf2_geometry_msgs import tf2_geometry_msgs

bridge = CvBridge()
config = Config()
MODE = config.MODE


class ArucoLocater:
    def __init__(self):
        r"""
        ``intrinsic``: camera intrinsic parameter matrix

        .. code-block:: python
            camera_matrix = ⎡ fx   0  cx ⎤
                            ⎢  0  fy  cy ⎥
                            ⎣  0   0   1 ⎦

        - ``fx`` and ``fy`` are the focal lengths in pixels
        - ``cx`` and ``cy`` are the coordinates of the principal point in pixels
            - they are usually the center of the image, i.e.
            ``cx = width / 2``, ``cy = height / 2``
        """
        self.distortion = np.zeros((1, 5))

        if MODE == "REAL":
            self.intrinsic = np.array([[554, 0, 320], [0, 554, 240], [0, 0, 1]])  # real cam
            # self.intrinsic = np.array([[2270, 0, 960], [0, 2270, 540.0], [0, 0, 1]])  # real cam
            self.image_path = "/image_raw"  # for cam on agv

        else:
            self.intrinsic = np.array([[2270.61, 0.0, 960.5], [0.0, 2270.61, 540.5], [0, 0, 1]])  # sim cam
            self.image_path = "/modulating_part/camera1/image_raw/"  # for cam in gazebo

        self.image_raw = None
        self.image_gray = None
        self.image_sub = rospy.Subscriber(self.image_path, Image, self.image_callback, queue_size=10)

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        self.rvec = None
        self.tvec = None
        self.alpha = None
        self.quat = None

    def image_callback(self, msg):
        self.image_raw = bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )  # transform ros image to opencv image
        self.image_gray = cv2.cvtColor(self.image_raw, cv2.COLOR_BGR2GRAY)
        # cv2.imshow("image", self.image_raw)
        # cv2.write("./data/image.jpg", self.image_raw)
        self.update_aruco_location()

    def estimate_pose(self, corners, length):
        points_3d = (
            np.array([[0, 0.5, -0.5], [0, 0.5, 0.5], [0, -0.5, 0.5], [0, -0.5, -0.5]], dtype=np.float32)
            * length
        )
        points_2d = np.array(corners[0], dtype=np.float32)
        _, rvec, tvec = cv2.solvePnP(points_3d, points_2d, self.intrinsic, self.distortion)
        return rvec, tvec

    def update_aruco_location(self):
        if self.image_gray is None:
            return
        # detect and update marker's id and corner
        corners, ids, _ = self.detector.detectMarkers(self.image_gray)
        # print corners and ids
        print("corners: ", corners)
        print("ids: ", ids)

        if ids is not None:
            self.corners = corners
            self.ids = ids
            if ids.size != 1:
                corners = corners[-1]
                ids = ids[-1]
            # get the rotation vector and translation vector
            rvec, tvec = self.estimate_pose(corners, 0.15)
            print("pose:", rvec, tvec)
            # get transform matrix
            R_camera_image = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
            R_image_aruco = np.zeros((4, 4), dtype=np.float32)
            # convert rotation vector to rotation matrix
            rot_mat = cv2.Rodrigues(rvec)
            # fill the rotation matrix
            R_image_aruco[:3, :3] = rot_mat[0]
            R_image_aruco[:3, 3] = tvec.flatten()
            R_image_aruco[3, 3] = 1
            # combine transform matrix
            R_camera_aruco = R_camera_image @ R_image_aruco
            # quaternion from rotation vector and translation vector
            self.quat = tf.transformations.quaternion_from_matrix(R_camera_aruco)
            self.tvec = R_camera_aruco[:3, 3].flatten()
            print(f"tvec:{self.tvec}")
        else:
            raise ValueError("No marker detected")


class plane_part_with_agv:
    def __init__(self, nodename):

        rospy.init_node(nodename)

        self.xiangjiLink = config.cameraLink
        self.agvLink = "agv_part/agv_Link"

        # self.modulatingLink = "modulating_Link"
        # self.platformLink = "platform_Link"
        # self.bottomLink = "bottom_Link"

        self.referenceLink = "agv_part/reference_Link"
        self.imageTargetLink = "agv_part/image_target1"

        self.bottomPlatformRel = np.array([0.0, 0.0, 0.614, 0.0, 0.0, 0.0, 1.0])

        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        self.aruco_locater = ArucoLocater()

    def lookup_transform_quaternion_vector(self, target_frame, source_frame, time, timeout):
        trans_ = self.tfBuffer.lookup_transform(target_frame, source_frame, time, timeout)
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
                    camImageTargetRel = np.concatenate(
                        (self.aruco_locater.tvec, self.aruco_locater.quat), axis=0
                    )
                else:
                    camImageTargetRel = np.array([1.6, 0, 0, 0, 0, 0, 1])

                newTrans = matrixHelper.compose_quaternion_vector(camImageTargetRel, imageTargetAgvRel)

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
        # rospy.spin()


if __name__ == "__main__":
    a = input("press any key to start...")
    service = plane_part_with_agv("digital_twining")
    service.run()
