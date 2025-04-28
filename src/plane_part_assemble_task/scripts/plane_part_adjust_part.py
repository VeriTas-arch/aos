#!/usr/bin/python3

import numpy as np
import rospy
import tf2_ros

from mathHelper import matrixHelper
from plane_part_assemble import MoveGroupPythonInterfaceTutorial


class plane_part_adjust_part:
    def __init__(self, nodename):
        self.planningInterface = MoveGroupPythonInterfaceTutorial()

        self.xiangjiLink = "xiangji_Link"
        self.modulatingLink = "modulating_Link"
        self.platformLink = "platform_Link"
        self.bottomLink = "bottom_Link"
        self.referenceLink = "agv_part/reference_Link"
        self.imageTargetLink = "agv_part/image_target1"
        self.agvLink = "agv_part/agv_Link"
        self.world = "world"

        self.bottomPlatformRel = np.array([0.0, 0.0, 0.614, 0.0, 0.0, 0.0, 1.0])

        self.tfBroadcaster = tf2_ros.TransformBroadcaster()

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

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
        modulatingPlatformRel = self.lookup_transform_quaternion_vector(
            self.modulatingLink, self.platformLink, rospy.Time(0), rospy.Duration(1.0)
        )

        try:
            # real state of the model
            referenceModulatingRel = self.lookup_transform_quaternion_vector(
                self.referenceLink,
                self.modulatingLink,
                rospy.Time(0),
                rospy.Duration(1.0),
            )

            # target state of the model
            referenceModulatingRel = np.array([-1.6, 0, 0, 0, 0, 0, 1])

            referenceWorldRel = self.lookup_transform_quaternion_vector(
                self.referenceLink, self.world, rospy.Time(0), rospy.Duration(1.0)
            )

            # worldPlatformRel = self.lookup_transform_quaternion_vector(
            #     self.world, self.platformLink, rospy.Time(0), rospy.Duration(1.0)
            # )

            referencePlatformRel = matrixHelper.compose_quaternion_vector(
                referenceModulatingRel, modulatingPlatformRel
            )

            worldPlatformVector = matrixHelper.getRelativeTransformVector(
                referenceWorldRel, referencePlatformRel
            )

            self.planningInterface.go_to_pose_goal(worldPlatformVector)

        except rospy.ROSInterruptException:
            return

        except KeyboardInterrupt:
            return

        rospy.spin()


if __name__ == "__main__":
    service = plane_part_adjust_part("plane_part_adjust_part")
    service.run()
