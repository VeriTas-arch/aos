#! /usr/bin/env python

import math
import os
from ctypes import sizeof

import numpy as np
import rospy
import stewartKin
import tf2_ros
import yaml
from beckhoff_device import beckhoff_device
from mathHelper import matrixHelper
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool, Float64MultiArray, Header
from stewart_virtual_model import StewartVirtualModel
from tf import TransformListener, transformations


class calibrate_stewart(object):
    def __init__(self, saved_file_name):

        self.device = beckhoff_device("192.168.1.103.1.1", "192.168.1.100")

        self.model = StewartVirtualModel()

        # self.joint_names = [
        #     "platform_x_joint",
        #     "platform_c_Joint",
        #     "platform_b_joint",
        #     "platform_a_joint",
        #     "platform_z_joint",
        #     "platform_y_joint"]

        # self.name_position_dict = {x:0.0 for x in self.joint_names}

        # there parameters can be written in yaml and load before running
        bottomRadius = 401.4 / 1000.0  # 401.4
        bottomNeiborAngle = 11.46 / 180.0 * math.pi  # 11.46deg
        bottomZoffset = 33.93 / 1000.0  # 33.93
        platformRadius = 319.25 / 1000.0  # 319.25
        platformNeiborAngle = 7.86 / 180.0 * math.pi  # 7.86
        platformZoffset = -35.0 / 1000.0  # -35.00

        self.initializeStandardStewart(
            bottomRadius,
            bottomNeiborAngle,
            bottomZoffset,
            platformRadius,
            platformNeiborAngle,
            platformZoffset,
        )

        # self.js = JointState()
        # self.js.header = Header()
        # self.js.name = [i for i in self.name_position_dict.keys()]
        # self.js.position = [self.name_position_dict[i] for i in self.js.name]
        # self.js.velocity = []
        # self.js.effort = []

        # self.pub = rospy.Publisher('sim_joint_states', JointState, queue_size=1)

        self.update_file_name = saved_file_name

        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf = tf2_ros.TransformListener(self.tf_buffer)

    def initializeStandardStewart(self, r1, a1, z1, r2, a2, z2):
        bottomAngleList = [
            a1 / 2,
            -a1 / 2 + math.pi * 2 / 3,
            a1 / 2 + math.pi * 2 / 3,
            -a1 / 2 - math.pi * 2 / 3,
            a1 / 2 - math.pi * 2 / 3,
            -a1 / 2,
        ]

        self.bottomJoints = np.array([[r1 * math.cos(i), r1 * math.sin(i), z1] for i in bottomAngleList])

        platformAngleList = [
            -a2 / 2 + math.pi / 3,
            a2 / 2 + math.pi / 3,
            -a2 / 2 + math.pi,
            a2 / 2 - math.pi,
            -a2 / 2 - math.pi / 3,
            a2 / 2 - math.pi / 3,
        ]

        self.platformJoints = np.array(
            [[r2 * math.cos(i), r2 * math.sin(i), z2] for i in platformAngleList]
        )

        self.stewartKinmatics = stewartKin.stewartKin(self.bottomJoints, self.platformJoints)

    # def publishActiveJointStates(self,activeNames):
    #     self.js.name = activeNames
    #     self.js.position = [self.name_position_dict[i] for i in activeNames]
    #     self.js.header.stamp = rospy.Time.now()
    #     self.pub.publish(self.js)

    # def enable_motor(self, enable = True):
    #     self.device.enableBeckhoffMotor(enable)

    def execute_calibration(self):
        # activeNames = self.joint_names
        # for i in activeNames:
        #     self.name_position_dict[i] = 0.0

        # self.publishActiveJointStates(activeNames)
        self.model.goto_zero_config()

        worldLink = "world"
        sourceLink = "bottom_Link"
        targetLink = "platform_Link"

        tqBottom = self.model.get_relative_transform(worldLink, sourceLink)
        # transBottom = self.tf_buffer.lookup_transform(worldLink,sourceLink,rospy.Time(),rospy.Duration(10.0))
        # matBottom = matrixHelper.getTransformVector(transBottom.transform)

        tqPlatform = self.model.get_relative_transform(worldLink, targetLink)
        # transPlatform = self.tf_buffer.lookup_transform(worldLink,targetLink,rospy.Time(),rospy.Duration(10.0))
        # matPlatform = matrixHelper.getTransformVector(transPlatform.transform)

        # tqBottom = [transBottom.transform.translation.x, transBottom.transform.translation.y, transBottom.transform.translation.z,
        #     transBottom.transform.rotation.x, transBottom.transform.rotation.y, transBottom.transform.rotation.z, transBottom.transform.rotation.w]
        # tqPlatform = [transPlatform.transform.translation.x, transPlatform.transform.translation.y, transPlatform.transform.translation.z,
        #     transPlatform.transform.rotation.x, transPlatform.transform.rotation.y, transPlatform.transform.rotation.z, transPlatform.transform.rotation.w]

        matRel = matrixHelper.getRelativeTransformMatrix(tqBottom, tqPlatform)

        basicLegLengths = self.stewartKinmatics.inverseKin(matRel)

        # measuredMatRel is required to calibrate the whole platform
        # controlLengths = self.device.readLegControlLength()
        measuredMatRel = matRel.copy()

        # controlLengths = np.array([0.0/1000.0] * 6)
        controlLengths = self.device.readLegControlLength()

        measuredLegLengths = self.stewartKinmatics.inverseKin(measuredMatRel)

        fixedLegLength = measuredLegLengths - controlLengths

        deltaLegLength = fixedLegLength - basicLegLengths

        print("fixed_leg_length:", fixedLegLength)
        print("delta_leg_length:", deltaLegLength)

        pfile = open(self.update_file_name, "w", encoding="utf-8")

        output = {"calibration": {}}
        output["calibration"][
            "bottom_transform"
        ] = tqBottom  # [transBottom.transform.translation.x, transBottom.transform.translation.y, transBottom.transform.translation.z,
        # transBottom.transform.rotation.x, transBottom.transform.rotation.y, transBottom.transform.rotation.z, transBottom.transform.rotation.w]
        output["calibration"][
            "platform_transform"
        ] = tqPlatform  # [transPlatform.transform.translation.x, transPlatform.transform.translation.y, transPlatform.transform.translation.z,
        # transPlatform.transform.rotation.x, transPlatform.transform.rotation.y, transPlatform.transform.rotation.z, transPlatform.transform.rotation.w]
        # output['calibration']['control_zero_point'] = controlZeroPoint.tolist()
        output["calibration"]["fixed_leg_length"] = fixedLegLength.tolist()
        output["calibration"]["platform_joints"] = self.platformJoints.tolist()
        output["calibration"]["bottom_joints"] = self.bottomJoints.tolist()

        yaml.dump(output, pfile)
        pfile.close()


if __name__ == "__main__":
    print("starting plane part stewart calibration process")
    rospy.init_node("calibrate_stewart")

    calibration_file = "./"
    if rospy.has_param("calibration_file"):
        calibration_file = rospy.get_param("calibration_file")

    server = calibrate_stewart(calibration_file)
    server.execute_calibration()
