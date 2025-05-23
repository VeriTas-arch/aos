#!/usr/bin/python3

import time

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import tf.transformations
from config import Config
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped, Pose, PoseStamped, TransformStamped, Twist
from mathHelper import matrixHelper
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray, Int8, MultiArrayDimension, MultiArrayLayout
from tf2_geometry_msgs import tf2_geometry_msgs

config = Config()
MODE = config.MODE


class modu_controller:
    def __init__(self):
        rospy.init_node("modu_controller", anonymous=True)

        self.modulatingLink = config.modulatingLink
        self.platformLink = config.platformLink
        self.bottomLink = config.bottomLink

        self.referenceLink = "agv_part/reference_Link"

        # self.xiangjiLink = "xiangji_Link"
        # self.imageTargetLink = "agv_part/image_target1"
        # self.agvLink = "agv_part/agv_Link"
        self.world = "world"

        self.tfBroadcaster = tf2_ros.TransformBroadcaster()
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # Transformation have three modes:
        # 0: Transform on all directions instead of x
        # 1: Transform on x axis
        # 2: Fine tuning
        self.trans_target = 0

        self.transform = None
        if MODE == "REAL":
            self.pub = rospy.Publisher(
                "/stewart_ptp_command", Float64MultiArray, queue_size=10
            )  # unit: meter
        else:
            self.pub = rospy.Publisher(
                "/modulating_part/targeting_controller/command", Float64MultiArray, queue_size=10
            )

        self.state_subscriber = rospy.Subscriber("/modu_ctrl_cmd", Int8, self.state_callback)
        self.state_publisher = rospy.Publisher(
            "/modu_state", Int8, queue_size=10
        )  # 0: not moving; 1: moving

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

    def solve_transform(self):
        try:
            # set initial transform for bottom and platform
            BottomLink_PlatformLink_Init = np.array(
                [
                    3.99101395e-09,
                    1.39181683e-11,
                    6.14489406e-01,
                    3.01312447e-07,
                    5.19605386e-08,
                    2.97431105e-07,
                    1.00000000e00,
                ]
            )
            # set target for the assignment
            ReferenceLink_ModulatingLink_Target = np.array([-1.65, 0, 0, 0, 0, 0, 1])
            # get transformation for ModulatingLink and Platform: a fixed transformation
            ModulatingLink_PlatformLink = self.lookup_transform_quaternion_vector(
                self.modulatingLink, self.platformLink, rospy.Time(0), rospy.Duration(1.0)
            )

            # current transformation for ReferenceLink and BottomLink
            ReferenceLink_BottomLink = self.lookup_transform_quaternion_vector(
                self.referenceLink, self.bottomLink, rospy.Time(0), rospy.Duration(1.0)
            )

            # combine two transformations: Modulating_Platform and Reference_Modulate
            ReferenceLink_PlatformLink = matrixHelper.compose_quaternion_vector(
                ReferenceLink_ModulatingLink_Target, ModulatingLink_PlatformLink
            )

            # calculate final transformation for plane control
            BottomLink_PlatformLink = matrixHelper.getRelativeTransformVector(
                ReferenceLink_BottomLink, ReferenceLink_PlatformLink
            )
            Final_Transform = matrixHelper.getRelativeTransformVector(
                BottomLink_PlatformLink_Init, BottomLink_PlatformLink
            )
            self.transform = Final_Transform

        except rospy.ROSInterruptException:
            return 0
        except KeyboardInterrupt:
            return 1

    def go_to_pos_simu(self, T_vector):
        # transform matrix into euler angle
        translation = T_vector[:3]
        translation_1 = np.array([0, T_vector[1], T_vector[2]])
        quaternion = T_vector[3:]
        r, p, y = tf.transformations.euler_from_quaternion(quaternion)
        euler_angles = [r, p / 5, y]

        # create data list
        safe_range = 0.14
        print("Calling go_to_pos_simu...")

        print(translation)
        print(euler_angles)
        if self.trans_target == 0:
            if np.linalg.norm(translation_1) < safe_range:  # (meter)
                data = list(translation) + list(euler_angles)
                data[0] = 0
                data[3] = 0
                data[4] = 0
                data[5] = 0
                if MODE == "REAL":
                    data.append(20)
            else:
                raise ValueError("OUT OF RANGE")

            self.trans_target = 1

        elif self.trans_target == 1:
            if np.linalg.norm(translation) < safe_range:  # (meter)
                data = list(translation) + list(euler_angles)
                if MODE == "REAL":
                    data.append(20)
            else:
                raise ValueError("OUT OF RANGE")

        # encapsulate data
        msg = Float64MultiArray()
        msg.data = data

        layout = MultiArrayLayout()
        layout.dim = [MultiArrayDimension(label="", size=0, stride=0)]
        layout.data_offset = 0
        msg.layout = layout

        # publish message
        rospy.Rate(10)  # 10hz

        rospy.loginfo("Publishing Transform: %s", msg.data)
        input("ready?")

        for _ in range(1):
            self.pub.publish(msg)
            time.sleep(0.01)

        print("running...")

    def state_callback(self, msg):
        print("callback...")
        print("get state", msg.data)
        state = Int8()
        sleep_time = 5  # (second)

        if msg.data == 1:
            state.data = 1
            for _ in range(20):
                self.state_publisher.publish(state)
                time.sleep(0.01)

            self.solve_transform()
            self.go_to_pos_simu(self.transform)
            rospy.loginfo(f"Finished sending first messages, waiting for {sleep_time} seconds...")
            time.sleep(sleep_time)
            state.data = 0
            for _ in range(20):
                self.state_publisher.publish(state)
                time.sleep(0.01)

        elif msg.data == 2:
            state.data = 0
            for _ in range(20):
                self.state_publisher.publish(state)
                time.sleep(0.01)

        elif msg.data == 3:
            state.data = 1
            for _ in range(20):
                self.state_publisher.publish(state)
                time.sleep(0.01)

            print("Start calculation on state 3")

            self.solve_transform()
            self.go_to_pos_simu(self.transform)
            rospy.loginfo(f"Finished sending second messages, waiting for {sleep_time} seconds...")
            time.sleep(sleep_time)
            state.data = 0
            self.state_publisher.publish(state)

        else:
            return 0


if __name__ == "__main__":
    try:
        controller = modu_controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
