#! /usr/bin/python3

import time

import actionlib
import matplotlib.pyplot as plt

# from control_msgs.msg import FollowJointTrajectoryActionGoal
import moveit_msgs.msg
import MXController
import numpy as np
import numpy.linalg as la
import rospy
import stewartKin
import yaml
from beckhoff_device import beckhoff_device_nofity
from mathHelper import matrixHelper
from std_msgs.msg import Bool, Float64MultiArray
from stewart_virtual_model import StewartVirtualModel

# from scipy.spatial.transform import Rotation as R
from tf import transformations


class plane_part_stewart_driver(object):
    def __init__(self, name):
        rospy.init_node(name)

        bottomJoints = rospy.get_param("/calibration/bottom_joints")
        platformJoints = rospy.get_param("/calibration/platform_joints")

        self.stewart = stewartKin.stewartKin(bottomJoints, platformJoints)

        # import the calibration result from parameter service
        self.stewart_calibration = {}
        self.stewart_calibration["bottom_transform"] = np.array(
            rospy.get_param("/calibration/bottom_transform")
        )
        self.stewart_calibration["platform_transform"] = np.array(
            rospy.get_param("/calibration/platform_transform")
        )
        # self.stewart_calibration['control_zero_point'] = np.array(rospy.get_param("/calibration/control_zero_point"))
        self.stewart_calibration["fixed_leg_length"] = np.array(
            rospy.get_param("/calibration/fixed_leg_length")
        )

        # create messages that are used to publish feedback/result
        self._feedback = moveit_msgs.msg.ExecuteTrajectoryFeedback()
        self._result = moveit_msgs.msg.ExecuteTrajectoryResult()

        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            moveit_msgs.msg.ExecuteTrajectoryAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()

        self._device = beckhoff_device_nofity("192.168.1.103.1.1", "192.168.11.110")
        self._device.disableFifo()

        self._model = StewartVirtualModel()
        # worldLink = "world"
        # sourceLink = "bottom_Link"
        # targetLink = "platform_Link"

        # tqBottom = self._model.get_relative_transform(worldLink,sourceLink)
        # tqPlatform = self._model.get_relative_transform(worldLink, targetLink)
        # self.bottom2Platform = matrixHelper.getRelativeTransformMatrix(tqBottom,tqPlatform)

        basicPlanner = MXController.trajectoryPlanner(self.stewart, self.stewart_calibration)
        self.bottom2Platform = basicPlanner.bottom2Platform

        self.jointStateController = MXController.JointStateController(
            self.stewart, self.stewart_calibration
        )

        self.enable_sub = rospy.Subscriber("/enable_motor", Bool, self.enable_cb)
        self.ptp_sub = rospy.Subscriber("/stewart_ptp_command", Float64MultiArray, self.ptp_cb)

    def enable_cb(self, msg):
        self._device.enableBeckhoffMotor(msg.data)

    def ptp_cb(self, msg):
        if len(msg.data) < 6:
            return

        if not self._device.isBeckhoffMotorEnabled():
            rospy.loginfo("motor is not enabled, unable to run")
            return

        this_f = msg.data
        mat_traj = transformations.euler_matrix(this_f[3], this_f[4], this_f[5], axes="rxyz")
        mat_traj[0:3, 3] = this_f[0:3]
        trans_mat_traj = self.bottom2Platform @ mat_traj

        if len(this_f) > 6:
            tv = max(5.0, this_f[6])
        else:
            tv = 5.0

        simuLegLength = self._device.readLegControlLength()
        self.ptpplanner = MXController.PtPController(
            self.stewart,
            self.stewart_calibration,
            simuLegLength,
            trans_mat_traj,
            timeInterval=tv,
            mode="Leg2Pose",
        )
        self.ptpplanner.ResetController()
        self.run_serial_trajectory([self.ptpplanner])

    def run_serial_trajectory(self, trajectory_list):
        self._device.restartFifo()
        if self._device.echoTrajectoryProc(trajectory_list, wait=0.0) > 0:
            time.sleep(0.2)
            self._device.startFifoRunning()

        while self._device.echoTrajectoryProc(trajectory_list) != 0:
            pass
        time.sleep(0.5)
        self._device.disableFifo()
        return

    def runCycle(self):
        r = rospy.Rate(5)
        platform2Bottom = transformations.inverse_matrix(self.bottom2Platform)

        ig = self.bottom2Platform
        while not rospy.is_shutdown():
            simuLegLength = self._device.readLegControlLength()
            trans_mat_traj = self.jointStateController.getJointState(simuLegLength, initGuess=ig)
            ig = trans_mat_traj

            mat_traj = platform2Bottom @ trans_mat_traj
            joint_states = mat_traj[0:3, 3].tolist()
            joint_angles = transformations.euler_from_matrix(mat_traj, axes="rxyz")
            joint_states.extend(joint_angles)

            for i in range(len(joint_states)):
                self._model.name_position_dict[self._model.joint_names[i]] = joint_states[i]
            self._model.publishActiveJointStates(self._model.joint_names)

            r.sleep()

    def execute_cb(self, goal):
        # helper variables
        success = True

        # append the seeds for the fibonacci sequence
        self._feedback.state = "setup"
        self._as.publish_feedback(self._feedback)

        # publish info to the console for the user
        rospy.loginfo("%s: with state %s" % (self._action_name, self._feedback.state))

        traj_points = goal.trajectory.joint_trajectory.points

        traj = np.empty((0, 7))

        first_frame_matrix = None
        for i in range(len(traj_points)):
            point = traj_points[i]

            this_t = point.time_from_start.secs + point.time_from_start.nsecs / 1e9
            this_f = point.positions

            mat_traj = transformations.euler_matrix(this_f[3], this_f[4], this_f[5], axes="rxyz")
            mat_traj[0:3, 3] = this_f[0:3]

            trans_mat_traj = self.bottom2Platform @ mat_traj
            if i == 0:
                first_frame_matrix = trans_mat_traj

            this_p = trans_mat_traj[0:3, 3]
            this_r = transformations.euler_from_matrix(trans_mat_traj, axes="rxyz")
            thisframe = np.concatenate([[this_t], this_p, this_r])
            traj = np.row_stack([traj, thisframe])

            # traj[i,1:] = np.matmul(self._calibration.transform_matrix(),traj[i,1:])
            # print(self._calibration.transform_matrix())
            # print(point.positions)

        # print(traj)

        simuLegLength = self._device.readLegControlLength()
        tempPlanner = MXController.JointStateController(self.stewart, self.stewart_calibration)
        firstLegLength = tempPlanner.GetLegLength(first_frame_matrix)
        delta = la.norm(firstLegLength - simuLegLength)

        if delta > 6e-3:
            print("large error from first frame found, move to first frame in ptp mode")
            print(simuLegLength)
            print(firstLegLength)

            tv = max(0.5, delta / 6e-3)
            self.ptpplanner = MXController.PtPController(
                self.stewart,
                self.stewart_calibration,
                simuLegLength,
                first_frame_matrix,
                timeInterval=tv,
                mode="Leg2Pose",
            )
            self.ptpplanner.ResetController()

            self.currentTrajectory = traj
            self.mxplanner = MXController.MXController(self.stewart, traj, self.stewart_calibration)
            self.mxplanner.CarculateMxTraj()
            self.mxplanner.ResetController()

            # start executing the action with MXController
            trajectory_list = [self.ptpplanner, self.mxplanner]
        else:
            if delta > 1e-5:
                print("deviation from first frame found, replace first frame")
                print(simuLegLength)
                print(firstLegLength)

                trans_mat_traj = tempPlanner.getJointState(simuLegLength)
                this_p = trans_mat_traj[0:3, 3]
                this_r = transformations.euler_from_matrix(trans_mat_traj, axes="rxyz")
                thisframe = np.concatenate([[0.0], this_p, this_r])
                traj[0, :] = thisframe

            self.currentTrajectory = traj
            self.mxplanner = MXController.MXController(self.stewart, traj, self.stewart_calibration)
            self.mxplanner.CarculateMxTraj()
            self.mxplanner.ResetController()

            # start executing the action with MXController
            trajectory_list = [self.mxplanner]

        self._device.restartFifo()
        if self._device.echoTrajectoryProc(trajectory_list, wait=0.0) > 0:
            time.sleep(0.2)
            self._device.startFifoRunning()

        running = True
        while running:
            result = self._device.echoTrajectoryProc(trajectory_list)
            # check that preempt has not been requested by the client
            if result == 0:
                running = False
            elif result > 0:
                if self._as.is_preempt_requested():
                    rospy.loginfo("%s: Preempted" % self._action_name)
                    self._as.set_preempted()
                    success = False
                    running = False

                self._feedback.state = "process frame" + str(result)

                # publish the feedback
                self._as.publish_feedback(self._feedback)

        time.sleep(0.5)
        self._device.disableFifo()

        if success:
            self._result.error_code.val = self._result.error_code.SUCCESS
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self._result)

    # def trajCB(self,msg):
    #     #print(msg.goal_id)
    #     traj_points = msg.goal.trajectory.joint_trajectory.points
    #     for i in traj_points:
    #         #print(i.positions)
    #         print(i.time_from_start.secs,i.time_from_start.nsecs)


if __name__ == "__main__":
    print("starting plane part stewart driver")
    server = plane_part_stewart_driver("stewart_driver")
    server.runCycle()
    rospy.spin()
