#!/usr/bin/env python
import math
import threading
import time
from ctypes import *

import mxOfflineSim
import numpy as np
import pyads
import roslibpy
import spatialmath.base as smb
from base_control import base_control
from Base_Modbus import Base
from basicKinematics import *

import MXController
import stewartKin

mobileStewart = stewartKin.stewartKin()

inclineAngle = 60 / 180 * math.pi
targetCenter = mobileStewart.getSuggestedPose(inclineAngle)

# parameters for procession movement
MatTargetPose = np.matmul(
    smb.transl(200 + targetCenter[0], 0, 19.8 + 521.5 + targetCenter[2]),
    smb.troty(inclineAngle),
)
processionAngle = 5 / 180 * math.pi
processionAngularV = 1 / 180 * math.pi  # 1 degree per second

inclineAngle = 60 / 180 * math.pi
targetCenter = mobileStewart.getSuggestedPose(inclineAngle)
MatTargetInitPose = np.matmul(
    smb.transl(200 + targetCenter[0], 0, 19.8 + 521.5 + targetCenter[2]),
    smb.troty(inclineAngle),
)

# PLC device initialization
pyads.PORT_SPS1 = 851
# plc = pyads.Connection('192.168.1.5.1.1',pyads.PORT_SPS1)
plc = pyads.Connection("169.254.76.155.1.1", pyads.PORT_SPS1)
plc.open()

# pyads.testserver

# cycle state data from twincat3 PLC
override = plc.read_by_name("MC_FIFO.stFifoInterface.rOverride", pyads.PLCTYPE_REAL)
print("override=", override)
override_flag = plc.read_by_name(
    "MC_FIFO.stFifoInterface.bSetOverride", pyads.PLCTYPE_BOOL
)
print("override=", override_flag)

adsCallParaListNames = [
    "MC_FIFO.stADSSharedMem.bReturnSignal",
    "MC_FIFO.stADSSharedMem.nFuntionCode",
    "MC_FIFO.stADSSharedMem.nReturnCode",
    "MC_FIFO.stADSSharedMem.nInputLen",
]
adsCallParaList = plc.read_list_by_name(adsCallParaListNames)
arrInput = plc.read_by_name(
    "MC_FIFO.stADSSharedMem.arrInput",
    pyads.PLCTYPE_LREAL * adsCallParaList["MC_FIFO.stADSSharedMem.nInputLen"],
)

AdsCallEvent = threading.Event()


@plc.notification(pyads.PLCTYPE_BOOL)
def adsCallback(handle, name, timestamp, value):
    print(
        '{1}: received new notitifiction for variable "{0}", value: {2}'.format(
            name, timestamp, value
        )
    )
    if value:
        AdsCallEvent.set()


adsCallSignal = plc.get_symbol("MC_FIFO.stADSSharedMem.bADSCallSignal")
adsCallSignal.auto_update = True
adsCallSignal.add_device_notification(adsCallback)

adsCallStatusNames = [
    ".g_sAxisArr[1].NcToPlc.ActPos",
    ".g_sAxisArr[2].NcToPlc.ActPos",
    ".g_sAxisArr[3].NcToPlc.ActPos",
    ".g_sAxisArr[4].NcToPlc.ActPos",
    ".g_sAxisArr[5].NcToPlc.ActPos",
    ".g_sAxisArr[6].NcToPlc.ActPos",
]


def CreateCurrentposController(currentLegLength):

    ptpController = MXController.PtPController(
        mobileStewart, currentLegLength, MatTargetInitPose, 10, "Leg2Pose"
    )
    ptpController.ResetController()
    return ptpController


def CreateInitposController():
    mx = mxOfflineSim.mxOfflineSim()
    mxctrl = MXController.MXController(mobileStewart, mx)
    mxctrl.SplitAGVStewart()
    mxctrl.CarculateMxTraj()
    mxctrl.ResetController()
    agvTraj = mxctrl.GetAGVTraj()
    initPoseInTraj = mxctrl.GetInitPoseOfStewart()

    initController = MXController.PtPController(
        mobileStewart, MatTargetInitPose, initPoseInTraj, 15, "Pose2Pose"
    )
    initController.ResetController()
    return initController, mxctrl, agvTraj


def echoTrajectoryProc(controllers):
    if AdsCallEvent.wait(0.01) and AdsCallEvent.is_set():
        adsCallParaList = plc.read_list_by_name(adsCallParaListNames)
        print(adsCallParaList)
        paraNum = adsCallParaList["MC_FIFO.stADSSharedMem.nInputLen"]
        if paraNum > 0:
            arrInput = plc.read_by_name(
                "MC_FIFO.stADSSharedMem.arrInput",
                pyads.PLCTYPE_LREAL
                * adsCallParaList["MC_FIFO.stADSSharedMem.nInputLen"],
            )
            print(arrInput)
        AdsCallEvent.clear()
        col = int(arrInput[0])
        row = int(arrInput[1])

        for i in controllers:
            memRet, dataLen = i.RunControlStep(col, row)
            if dataLen != 0:
                break

        if dataLen > 0:
            plc.write_by_name(
                "MC_FIFO.stADSSharedMem.arrOutput",
                memRet,
                pyads.PLCTYPE_LREAL * dataLen,
            )
        adsCallParaList["MC_FIFO.stADSSharedMem.bReturnSignal"] = 1
        adsCallParaList["MC_FIFO.stADSSharedMem.nReturnCode"] = dataLen
        plc.write_list_by_name(adsCallParaList)

    else:
        return -1
    return dataLen


AdsCallEvent_base = threading.Event()


def process_data(dir="../data/base_path.txt"):
    """
    needs to be rewritten according to the relationship between the trajectory and the motion capture
    """
    time = []
    x_pos = []
    y_pos = []
    theta = []
    with open(dir, "r") as f:
        for line in f.readlines():
            data = list(map(float, line.split()))
            time.append(data[0])
            y_pos.append(-data[1] + 4000)
            x_pos.append(data[2] - 4000)
            theta.append(-90)
    return time, x_pos, y_pos, theta


class CarculatePTPThread(threading.Thread):
    def __init__(self, threadID, name, currentLegLength):
        # threading.Thread.__init__(self)
        super(CarculatePTPThread, self).__init__()
        self.threadID = threadID
        self.name = name
        self.currLegLength = currentLegLength
        self.isover = False

    def run(self):
        self.result = CreateCurrentposController(self.currLegLength)
        self.isover = True

    def get_result(self):
        # threading.Thread.join(self)
        try:
            return self.result
        except Exception:
            return None


class base_signal:
    def __init__(self):
        self.go_initial_pos = False
        self.start_trajectory = False
        self.pause = False
        self.resume = False
        self.signal = ""

    def initial_callback(self, msg):
        self.go_initial_pos = msg.data
        if self.go_initial_pos == True:
            # print('fx base is moving to the initial position')
            print("MX base is moving to the initial position")

    def start_callback(self, msg):
        self.start_trajectory = msg.data
        if self.start_trajectory == True:
            # print('fx base is moving the given trajectory')
            print("MX base is moving the given trajectory")

    def signal_callback(self, msg):
        self.signal = msg["data"]
        print(self.signal)
        AdsCallEvent_base.set()


if __name__ == "__main__":
    # ip of the base,136 for MX and 135 for FX
    mx_base = Base("172.31.1.136", "502")

    # an object of base_control class
    bc = base_control(1.0, 1.0, 0.1)
    bs = base_signal()

    # the directory that saves the base path
    dir = "../data/base_path.txt"
    # time_file,x,y,theta=process_data(dir)
    time_file = []
    x = []
    y = []
    theta = []
    initctrl, trajctrl, base_traj = CreateInitposController()

    num = np.shape(base_traj)[0]
    for i in range(0, num):
        time_file.append(base_traj[i, 0])
        x.append(base_traj[i, 1])
        y.append(base_traj[i, 2])
        theta.append(base_traj[i, 3])
    bc.max_time = time_file[num - 1]

    # put the path data to base control object
    bc.get_data(time_file, x, y, theta)

    initial_pose_x = bc.x_spline.get_position(0.01)
    initial_pose_y = bc.y_spline.get_position(0.01)
    initial_pose_theta = bc.theta_spline.get_position(0.01)
    print("initial position is ")
    print("x: " + str(initial_pose_x))
    print("y: " + str(initial_pose_y))
    print("theta: " + str(initial_pose_theta))
    print("max simulation time: " + str(bc.max_time))

    # ros init
    # rospy.init_node('paper_trajectory')
    rospy = roslibpy.Ros("192.168.2.76", port=9090)
    rospy.run()
    state_sub = roslibpy.Topic(rospy, "MX_msg", "geometry_msgs/PoseStamped")
    state_sub.subscribe(bc.pose_callback)

    # subscribe the state
    signal_sub = roslibpy.Topic(rospy, "MX_signal", "std_msgs/String")
    signal_sub.subscribe(bs.signal_callback)

    # publish the state
    state_pub = roslibpy.Topic(rospy, "MX_state", "std_msgs/String")

    # publish the power of battery
    power_pub = roslibpy.Topic(rospy, "MX_power", "std_msgs/Float32")

    with open("../data/actual_base_data.txt", "w") as f:
        f.write(
            "the following data follows: time, pos_d.x, pos_d.y, pos_d.theta,pos_a.x, pos_a.y, pos_a.theta, z \r\n"
        )

    hz = 10
    last_time = 0.0
    count = 0.0

    # startint the main loop
    print("starting the main loop to control the base")
    count = 0

    base_initialized = False
    # init_delta_time=time.time() - bc.current_posestamped_sec
    has_motion_capture_data = True
    state = "null"
    waiting_start_time = 0.0

    substate = ""
    startt = 0.0
    # flag=''
    plc.write_by_name("MC_FIFO.externalEnable", False, pyads.PLCTYPE_BOOL)
    deltaMxX = 0.0
    deltaMxY = 0.0
    while rospy.is_connected:
        timetest_stt = time.time()
        state_pub.publish(roslibpy.Message({"data": state}))
        power_pub.publish(roslibpy.Message({"data": mx_base.get_power()}))
        # delta_time=time.time()-bc.current_posestamped_sec

        # enable motors, state changes to 'motor_enabled'
        if state == "null":
            if AdsCallEvent_base.wait(0.1) and AdsCallEvent_base.is_set():
                if bs.signal == "enable_the_motor":
                    # stewart platform enable
                    plc.write_by_name(
                        "MC_FIFO.externalEnable", True, pyads.PLCTYPE_BOOL
                    )
                    plc.write_by_name(
                        "MC_FIFO.stFifoInterface.bStart", False, pyads.PLCTYPE_BOOL
                    )
                    plc.write_by_name(
                        "MC_FIFO.stFifoInterface.bChannelStart",
                        False,
                        pyads.PLCTYPE_BOOL,
                    )
                    plc.write_by_name("Tc3_Motion.set_do", False, pyads.PLCTYPE_BOOL)
                    plc.write_by_name("Tc3_Motion.move_vdo", False, pyads.PLCTYPE_BOOL)
                    plc.write_by_name("Tc3_Motion.stop_do", True, pyads.PLCTYPE_BOOL)
                    mx_base.set_base_to_start_mode()
                    state = "motor_enabled"
                AdsCallEvent_base.clear()

        # go to initial position, and state changes to 'initialized'
        # if signal.go_initial_pos==True and base_initialized==False:
        elif state == "motor_enabled":

            if AdsCallEvent_base.wait(0.1) and AdsCallEvent_base.is_set():
                if bs.signal == "start_the_initialization":
                    plc.write_by_name("Tc3_Motion.stop_do", False, pyads.PLCTYPE_BOOL)
                    plc.write_by_name("Tc3_Motion.set_do", True, pyads.PLCTYPE_BOOL)
                    initctrl.ResetController()
                    trajctrl.ResetController()
                    bc.time_from_start = 0.0
                    count = 0
                    bc.start = False
                    bc.tra_finished = False
                    substate = "sub_init"
                    state = "initializing"
                    adsCallStatus = plc.read_list_by_name(adsCallStatusNames)
                    currentLegLength = list(adsCallStatus.values())
                    thread1 = CarculatePTPThread(
                        1, "Thread-curr2init", currentLegLength
                    )
                    thread1.start()
                AdsCallEvent_base.clear()

        elif state == "initializing":
            if substate == "sub_init":
                if thread1.isover == True:
                    ptpctrl = thread1.get_result()
                    thread1.join()
                    thread1 = None
                    plc.write_by_name(
                        "MC_FIFO.stFifoInterface.bStart", True, pyads.PLCTYPE_BOOL
                    )
                    substate = "sub_start_echo"

            elif substate == "sub_start_echo":
                retval = echoTrajectoryProc([ptpctrl, initctrl])
                # retval = echoTrajectoryProc([ptpctrl])
                if retval > 0:
                    startt = time.time()
                    substate = "sub_wait_start"

            elif substate == "sub_wait_start":
                if time.time() - startt > 1.0:
                    plc.write_by_name(
                        "MC_FIFO.stFifoInterface.bChannelStart",
                        True,
                        pyads.PLCTYPE_BOOL,
                    )
                    plc.write_by_name(
                        "MC_FIFO.stFifoInterface.bSetOverride", True, pyads.PLCTYPE_BOOL
                    )
                    substate = "sub_proc_echo"

            elif substate == "sub_proc_echo":
                retval = echoTrajectoryProc([ptpctrl, initctrl])
                # retval = echoTrajectoryProc([ptpctrl])
                if retval == 0:
                    plc.write_by_name(
                        "MC_FIFO.stFifoInterface.bSetOverride",
                        False,
                        pyads.PLCTYPE_BOOL,
                    )
                    plc.write_by_name(
                        "MC_FIFO.stFifoInterface.bChannelStart",
                        False,
                        pyads.PLCTYPE_BOOL,
                    )
                    substate = "sub_wait_complete"

            elif substate == "sub_wait_complete":
                isFinished = plc.read_by_name(
                    "MC_FIFO.stFifoInterface.bFinished", pyads.PLCTYPE_BOOL
                )
                if isFinished:
                    # if(True):
                    plc.write_by_name(
                        "MC_FIFO.stFifoInterface.bStart", False, pyads.PLCTYPE_BOOL
                    )
                    substate = "sub_init_wait_start"

                    # flag = 'wait_start'

            vel_linear_x = 0.0
            vel_linear_y = 0.0
            vel_angular_z = 0.0

            actualTrans = (
                smb.transl(bc.current_pose2d_x, bc.current_pose2d_y, bc.current_pose_z)
                @ smb.trotx(bc.thetaX)
                @ smb.troty(bc.thetaY)
                @ smb.trotz(bc.thetaZ)
            )
            TDeltaCenterOfAGV = np.array(
                [
                    [0.0, 0.0, -1.0, 0.0],
                    [0.0, -1.0, 0.0, -425],
                    [-1.0, 0.0, 0.0, -334],
                    [0.0, 0.0, 0.0, 1.0],
                ]
            )
            actualTrans = actualTrans @ TDeltaCenterOfAGV

            actualTransTheta = smb.tr2rpy(actualTrans)
            actualTrans2D = smb.transl2(
                actualTrans[0, 3], actualTrans[1, 3]
            ) @ smb.trot2(actualTransTheta[2])

            TDeltaMxToAGV = np.array(
                [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
            )

            desiredTrans = (
                smb.transl2(initial_pose_x, initial_pose_y)
                @ smb.trot2(initial_pose_theta / 180.0 * math.pi)
                @ TDeltaMxToAGV
            )

            actualTrans2DRot = np.array(
                [
                    [actualTrans2D[0, 0], actualTrans2D[1, 0]],
                    [actualTrans2D[0, 1], actualTrans2D[1, 1]],
                ]
            )
            actualTrans2DTrans = -actualTrans2DRot @ np.array(
                [[actualTrans2D[0, 2]], [actualTrans2D[1, 2]]]
            )
            actualTrans2DInv = np.array(
                [
                    [
                        actualTrans2DRot[0, 0],
                        actualTrans2DRot[0, 1],
                        actualTrans2DTrans[0, 0],
                    ],
                    [
                        actualTrans2DRot[1, 0],
                        actualTrans2DRot[1, 1],
                        actualTrans2DTrans[1, 0],
                    ],
                    [0.0, 0.0, 1.0],
                ]
            )

            errorMarrix = actualTrans2DInv @ desiredTrans

            base_error_x = errorMarrix[0, 2]
            base_error_y = errorMarrix[1, 2]
            base_error_theta_rpy = smb.tr2rpy(errorMarrix)
            base_error_theta = base_error_theta_rpy[2] * 180.0 / math.pi

            if (
                abs(base_error_x) > 10
                or abs(base_error_y) > 10
                or abs(base_error_theta) > 5
            ):
                if has_motion_capture_data:  # and bc.time_from_start !=0:
                    if abs(base_error_y) > 10:
                        if base_error_y > 0:
                            vel_linear_y = 1
                        else:
                            vel_linear_y = -1
                    if abs(base_error_x) > 10:
                        if base_error_x > 0:
                            vel_linear_x = 1
                        else:
                            vel_linear_x = -1
                    if abs(base_error_theta) > 5:
                        vel_linear_x = 0
                        vel_linear_y = 0
                        if base_error_theta > 0:
                            vel_angular_z = 1
                        else:
                            vel_angular_z = -1
                    print(base_error_theta, vel_angular_z)
            elif (
                abs(base_error_x) < 10
                and abs(base_error_y) < 10
                and abs(base_error_theta) < 5
            ):
                if substate == "sub_init_wait_start":
                    state = "initialized"

                    print("initilation is complete translate to initialized")

            mx_base.set_XYTHETA_velocity(vel_linear_x, vel_linear_y, vel_angular_z)

        elif state == "initialized":
            if AdsCallEvent_base.wait(0.1) and AdsCallEvent_base.is_set():
                if bs.signal == "move_the_given_path":
                    state = "waiting_for_tra"
                    substate = "sub_init_waiting_for_tra"
                    waiting_start_time = time.time()
                AdsCallEvent_base.clear()

        elif state == "waiting_for_tra":
            if substate == "sub_init_waiting_for_tra":
                plc.write_by_name(
                    "MC_FIFO.stFifoInterface.bStart", True, pyads.PLCTYPE_BOOL
                )
                plc.write_by_name("Tc3_Motion.max_velocity", 50.0, pyads.PLCTYPE_LREAL)
                substate = "sub_start_echo"

            elif substate == "sub_start_echo":
                retval = echoTrajectoryProc([trajctrl])
                if retval > 0:
                    startt = time.time()
                    substate = "sub_wait_start"

            elif substate == "sub_wait_start":
                if time.time() - startt > 1.0:
                    plc.write_by_name("Tc3_Motion.set_do", False, pyads.PLCTYPE_BOOL)
                    plc.write_by_name("Tc3_Motion.move_vdo", True, pyads.PLCTYPE_BOOL)
                    # plc.write_by_name('MC_FIFO.stFifoInterface.bChannelStart',True,pyads.PLCTYPE_BOOL)
                    substate = "sub_proc_echo"
                    # flag = 'run_traj'

            if time.time() - waiting_start_time > 10.0 and substate == "sub_proc_echo":
                plc.write_by_name(
                    "MC_FIFO.stFifoInterface.bChannelStart", True, pyads.PLCTYPE_BOOL
                )
                state = "run_to_the_target"
                # print(time.time(),waiting_start_time)
                # bc.init_time=0.0

        # move along the given trajectory
        elif state == "run_to_the_target":
            if substate == "sub_proc_echo":
                retval = echoTrajectoryProc([trajctrl])
                if retval == 0:
                    plc.write_by_name(
                        "MC_FIFO.stFifoInterface.bChannelStart",
                        False,
                        pyads.PLCTYPE_BOOL,
                    )
                    substate = "sub_wait_complete"

            elif substate == "sub_wait_complete":
                isFinished = plc.read_by_name(
                    "MC_FIFO.stFifoInterface.bFinished", pyads.PLCTYPE_BOOL
                )
                if isFinished:
                    # if(True):
                    plc.write_by_name(
                        "MC_FIFO.stFifoInterface.bStart", False, pyads.PLCTYPE_BOOL
                    )
                    plc.write_by_name("Tc3_Motion.move_vdo", False, pyads.PLCTYPE_BOOL)
                    plc.write_by_name("Tc3_Motion.stop_do", True, pyads.PLCTYPE_BOOL)
                    # plc.write_by_name('Tc3_Motion.set_do',True,pyads.PLCTYPE_BOOL)
                    substate = "sub_finished"
                    # flag = 'servo_enabled'

            # intialize time
            if count == 0:
                # start_time=time.time()
                bc.init_time = 0.0
                bc.start = True

            if bc.time_from_start > count * 10:
                print("current simulation time: " + str(count * 10))
                count += 1

            # current simulation time
            # if time.time()-start_time > bc.max_time - 0.01:
            if bc.time_from_start > bc.max_time - 0.01:
                print("task compeleted!")
                bc.tra_finished = True
                if substate == "sub_finished":
                    state = "motor_enabled"
                    mx_base.set_XYTHETA_velocity(0.0, 0.0, 0.0)
                continue

            if bc.time_from_start < bc.max_time - 0.01:
                # desired velocity
                # vel_d=bc.compute_desired_velocity(time.time()-start_time)
                vel_d = bc.compute_desired_velocity(bc.time_from_start)
                pos_d = bc.compute_desired_position(bc.time_from_start)
                pos_a = [
                    bc.current_pose2d_x,
                    bc.current_pose2d_y,
                    bc.current_pose2d_theta,
                ]
                vel_cl_linear_x = 0.0
                vel_cl_linear_y = 0.0
                if has_motion_capture_data:
                    # controlled velocity
                    # move to the center of MX
                    # deltaMxX+=(-math.sin(bc.current_pose2d_theta*math.pi/180)*343.014-math.cos(bc.current_pose2d_theta*math.pi/180)*235)
                    # deltaMxY+=(+math.cos(bc.current_pose2d_theta*math.pi/180)*343.014-math.sin(bc.current_pose2d_theta*math.pi/180)*235)

                    actualTrans = (
                        smb.transl(
                            bc.current_pose2d_x, bc.current_pose2d_y, bc.current_pose_z
                        )
                        @ smb.trotx(bc.thetaX)
                        @ smb.troty(bc.thetaY)
                        @ smb.trotz(bc.thetaZ)
                    )
                    actualTrans = actualTrans @ TDeltaCenterOfAGV

                    actualTransTheta = smb.tr2rpy(actualTrans)
                    actualTrans2D = smb.transl2(
                        actualTrans[0, 3], actualTrans[1, 3]
                    ) @ smb.trot2(actualTransTheta[2])

                    desiredTrans = (
                        smb.transl2(pos_d[0], pos_d[1])
                        @ smb.trot2(pos_d[2] / 180.0 * math.pi)
                        @ TDeltaMxToAGV
                    )

                    actualTrans2DRot = np.array(
                        [
                            [actualTrans2D[0, 0], actualTrans2D[1, 0]],
                            [actualTrans2D[0, 1], actualTrans2D[1, 1]],
                        ]
                    )
                    actualTrans2DTrans = -actualTrans2DRot @ np.array(
                        [[actualTrans2D[0, 2]], [actualTrans2D[1, 2]]]
                    )
                    actualTrans2DInv = np.array(
                        [
                            [
                                actualTrans2DRot[0, 0],
                                actualTrans2DRot[0, 1],
                                actualTrans2DTrans[0, 0],
                            ],
                            [
                                actualTrans2DRot[1, 0],
                                actualTrans2DRot[1, 1],
                                actualTrans2DTrans[1, 0],
                            ],
                            [0.0, 0.0, 1.0],
                        ]
                    )

                    errorMarrix = actualTrans2DInv @ desiredTrans

                    base_error_theta_rpy = smb.tr2rpy(errorMarrix)
                    base_error_theta = base_error_theta_rpy[2] * 180.0 / math.pi

                    error_p = [errorMarrix[0, 2], errorMarrix[1, 2], base_error_theta]

                    vel_cl = bc.compute_control_velocity(vel_d, error_p)

                    # print(desiredTrans)
                    print(error_p, vel_cl)

                    if vel_cl_linear_x > 5:
                        vel_cl_linear_x = 5
                    elif vel_cl_linear_x < -5:
                        vel_cl_linear_x = -5

                    if vel_cl_linear_y > 5:
                        vel_cl_linear_y = 5
                    elif vel_cl_linear_y < -5:
                        vel_cl_linear_y = -5
                else:
                    vel_cl = vel_d
                # print('vel_x '+str(vel_cl[0])+' '+'vel_y '+str(vel_cl[1])+'\r\n')
                # print('desired_x '+str(pos_d[0])+' '+'desired_y '+ str(pos_d[1])+ ' ' + 'desired_theta' + str(pos_d[2])+' ' +'\r\n')
                mx_base.set_XYTHETA_velocity(vel_cl[0], vel_cl[1], vel_cl[2])

                # with open('../data/actual_base_data.txt','a') as f:
                #     f.write(str(bc.time_from_start)+' '+str(pos_d[0])+' '+str(pos_d[1])+\
                #         ' '+str(pos_d[2])+' '+str(pos_a[0])+' '+str(pos_a[1])+' '+str(pos_a[2])+'\r\n')
                #' '+str(bc.current_posestamped['pose']['position']['z'])+'\r\n')
        time.sleep(0.01)
        mx_base.read_state_once()
        mx_base.keep_heart_beating()


"""
            if AdsCallEvent_base.wait(0.01) and AdsCallEvent_base.is_set():
                AdsCallEvent_base.clear()
                if bs.signal == 'pause':
                    state='run_to_the_target_paused'

                elif bs.signal == 'kill':
                    state='null'
                    print('task is killed!')
                    exit(0)

        elif state == 'run_to_the_target_paused':
            if AdsCallEvent_base.wait(0.1) and AdsCallEvent_base.is_set():
                if bs.signal == 'resume':
                    state='run_to_the_target'
                AdsCallEvent_base.clear()

        elif state == 'initializing_paused':
            if AdsCallEvent_base.wait(0.1) and AdsCallEvent_base.is_set():
                if bs.signal=='initializing':
                    state='motor_enabled'
                AdsCallEvent_base.clear()
"""
