#!/usr/bin/python3

import roslib
import rospy
import math
import tf
import tf2_ros
from geometry_msgs.msg import Pose, Twist, TransformStamped, PointStamped, PoseStamped
import time
import numpy as np
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
import copy
from tf2_geometry_msgs import tf2_geometry_msgs
from mathHelper import matrixHelper
from plane_part_assemble import MoveGroupPythonInterfaceTutorial

class plane_part_adjust_part():
    def __init__(self,nodename):
        # rospy.init_node(nodename)
        self.planningInterface = MoveGroupPythonInterfaceTutorial()

        # self.xiangjiLink = "modulating_part/xiangji_Link"
        # self.modulatingLink = "modulating_part/modulating_Link"
        self.xiangjiLink = "xiangji_Link"
        self.modulatingLink = "modulating_Link"
        self.platformLink = "platform_Link"
        self.bottomLink = "bottom_Link"
        self.referenceLink = "agv_part/reference_Link"
        self.imageTargetLink = "agv_part/image_target1"
        self.agvLink="agv_part/agv_Link"
        self.world="world"

        self.bottomPlatformRel = np.array([0.0,0.0,0.614,0.0,0.0,0.0,1.0])

        self.tfBroadcaster =tf2_ros.TransformBroadcaster()

        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def lookup_transform_quaternion_vector(self,target_frame,source_frame,time,timeout):
        trans_ = self.tfBuffer.lookup_transform(target_frame, source_frame, time, timeout)
        return np.array([
            trans_.transform.translation.x,
            trans_.transform.translation.y,
            trans_.transform.translation.z,
            trans_.transform.rotation.x,
            trans_.transform.rotation.y,
            trans_.transform.rotation.z,
            trans_.transform.rotation.w,
            ])

    def run(self):
        # rate = rospy.Rate(10)
        # trans = TransformStamped()

        # imageTargetAgvRel = self.lookup_transform_quaternion_vector(self.imageTargetLink, self.agvLink, rospy.Time(0), rospy.Duration(1.0))
        # # imageTargetAgvRel = np.array([
        # #     imageTargetAgv.transform.translation.x,
        # #     imageTargetAgv.transform.translation.y,
        # #     imageTargetAgv.transform.translation.z,
        # #     imageTargetAgv.transform.rotation.x,
        # #     imageTargetAgv.transform.rotation.y,
        # #     imageTargetAgv.transform.rotation.z,
        # #     imageTargetAgv.transform.rotation.w,
        # #     ])

        modulatingPlatformRel = self.lookup_transform_quaternion_vector(self.modulatingLink, self.platformLink, rospy.Time(0), rospy.Duration(1.0))
        # modulatingPlatform = self.tfBuffer.lookup_transform(self.modulatingLink, self.platformLink, rospy.Time(0), rospy.Duration(1.0))
        # modulatingPlatformRel = np.array([
        #     modulatingPlatform.transform.translation.x,
        #     modulatingPlatform.transform.translation.y,
        #     modulatingPlatform.transform.translation.z,
        #     modulatingPlatform.transform.rotation.x,
        #     modulatingPlatform.transform.rotation.y,
        #     modulatingPlatform.transform.rotation.z,
        #     modulatingPlatform.transform.rotation.w,
        #     ])

        try:
            #detection by aruco tags and gives quaternion from rotation vector and translation vector
            #xiangjiImageTargetRel = np.array([1.0,0.0,0.0,0.0,0.0,0.0,1.0])

            #quaternion multipy to indicate coordinates transformation
            # newTrans = tf.transformations.quaternion_multiply(xiangjiImageTargetRel[3:],imageTargetAgvRel[3:])

            # p = np.copy(xiangjiImageTargetRel[0:3])
            # purep = np.append(p , 0.0)

            # newp = tf.transformations.quaternion_multiply(
            #     tf.transformations.quaternion_multiply(newTrans, purep),
            #     tf.transformations.quaternion_inverse(newTrans)
            #     )

            # newp = newp[0:3]+imageTargetAgvRel[0:3]

            # newTrans = matrixHelper.compose_quaternion_vector(xiangjiImageTargetRel,imageTargetAgvRel)

            # #publish the tf topic to make corrected pose of two robots
            # trans.header.frame_id = self.xiangjiLink 
            # trans.child_frame_id = self.agvLink

            # trans.header.stamp = rospy.Time.now()

            # trans.transform.translation.x = newTrans[0]
            # trans.transform.translation.y = newTrans[1]
            # trans.transform.translation.z = newTrans[2]

            # trans.transform.rotation.x = newTrans[3]
            # trans.transform.rotation.y = newTrans[4]
            # trans.transform.rotation.z = newTrans[5]
            # trans.transform.rotation.w = newTrans[6]

            # self.tfBroadcaster.sendTransform(trans)

            #real state of the model
            referenceModulatingRel = self.lookup_transform_quaternion_vector(self.referenceLink, self.modulatingLink, rospy.Time(0), rospy.Duration(1.0))

            #target state of the model
            referenceModulatingRel = np.array([-1.6,0,0,0,0,0,1])

            # referenceBottomRel = self.lookup_transform_quaternion_vector(self.referenceLink, self.bottomLink, rospy.Time(0), rospy.Duration(1.0))
            referenceWorldRel = self.lookup_transform_quaternion_vector(self.referenceLink, self.world, rospy.Time(0), rospy.Duration(1.0))
            # referenceModulatingRel = np.array([
            #     referenceModulatingRel.transform.translation.x,
            #     referenceModulatingRel.transform.translation.y,
            #     referenceModulatingRel.transform.translation.z,
            #     referenceModulatingRel.transform.rotation.x,
            #     referenceModulatingRel.transform.rotation.y,
            #     referenceModulatingRel.transform.rotation.z,
            #     referenceModulatingRel.transform.rotation.w,
            #     ])
            # referencePlatformRel = matrixHelper.compose_quaternion_vector(referenceBottomRel, referencePlatformRel)
            worldPlatformRel=self.lookup_transform_quaternion_vector(self.world, self.platformLink, rospy.Time(0), rospy.Duration(1.0))

            referencePlatformRel = matrixHelper.compose_quaternion_vector(referenceModulatingRel, modulatingPlatformRel)
            # bottomPlatformVector=matrixHelper.getRelativeTransformVector(referenceBottomRel, referencePlatformRel)
            
            worldPlatformVector=matrixHelper.getRelativeTransformVector(referenceWorldRel, referencePlatformRel)

            # print(bottomPlatformVector)

            #self.planningInterface.go_to_joint_state()
            self.planningInterface.go_to_pose_goal(worldPlatformVector)

        except rospy.ROSInterruptException:
            return

        except KeyboardInterrupt:
            return
        
        rospy.spin()

if __name__ == '__main__':
    service = plane_part_adjust_part('plane_part_adjust_part')
    service.run()
