import math
import time

import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as la
import numpy.matlib as ml
import quaternion
import roboticstoolbox as rtb
import rospy
import scipy.interpolate as spi
import spatialmath.base as smb
# import roslibpy
# import pyads
import stewartKin
from beckhoff_device import beckhoff_device_nofity
from mathHelper import matrixHelper


class trajectoryPlanner(object):
    def __init__(self, stewart, stewart_calibration):
        self.stewart = stewart

        self.bottom2Platform = matrixHelper.getRelativeTransformMatrix(
            stewart_calibration["bottom_transform"], stewart_calibration["platform_transform"]
        )

        # self.control_zero_point = stewart_calibration['control_zero_point']
        self.fixed_leg_length = stewart_calibration["fixed_leg_length"]

    def GetLegLength(self, thisPose):
        thisLegLen = self.stewart.inverseKin(thisPose)
        return thisLegLen - self.fixed_leg_length


class JointStateController(trajectoryPlanner):
    def __init__(self, stewart, stewart_calibration):
        super(JointStateController, self).__init__(stewart, stewart_calibration)

    def getJointState(self, startConf, initGuess=None):
        if initGuess is None:
            initGuess = self.bottom2Platform

        currPose, iterc, deltaLen = self.stewart.forwardKin(
            startConf + self.fixed_leg_length, initGuess, eps=3e-6, maxiter=20
        )

        return currPose


class PtPController(trajectoryPlanner):
    def __init__(
        self, stewart, stewart_calibration, startConf, tgtConf, timeInterval=5.0, mode="Pose2Pose"
    ):
        super(PtPController, self).__init__(stewart, stewart_calibration)

        # jointLength0, MatTargetInitPose =stewart.zeroLegLength()

        if mode == "Leg2Pose" or mode == "Leg2Leg":
            currPose, iterc, deltaLen = self.stewart.forwardKin(
                startConf + self.fixed_leg_length, self.bottom2Platform, eps=3e-6, maxiter=20
            )
        else:
            currPose = startConf

        if mode == "Leg2Leg":
            tgtPose, iterc, deltaLen = self.stewart.forwardKin(
                tgtConf + self.fixed_leg_length, self.bottom2Platform, eps=3e-6, maxiter=20
            )
        else:
            tgtPose = tgtConf

        _, result = self.trajectoryPlanningPtp(currPose, tgtPose, timeInterval)

        self.buffer = result - self.fixed_leg_length
        self.framePointer = 0

    def trajectoryPlanningPtp(self, MatTargetInitPose, MatTargetPose, interval, tick=0.01):
        # p2p planning algorithm from the initial position to the starting point
        timeTraj = rtb.tools.trajectory.trapezoidal(1.0e-10, 1.0 - 1e-10, np.arange(0, interval, tick))

        # interpolation with quaternion
        qTargetInitPose = smb.quaternions.r2q(MatTargetInitPose[0:3, 0:3])
        qTargetPose = smb.quaternions.r2q(MatTargetPose[0:3, 0:3])
        tTargetInitPose = MatTargetInitPose[0:3, 3]
        tTargetPose = MatTargetPose[0:3, 3]
        result = np.empty([0, 6])
        startt = time.process_time()
        lastPose = np.copy(MatTargetInitPose)
        for i in timeTraj.q:
            thisq = smb.quaternions.qslerp(qTargetInitPose, qTargetPose, i, True)

            thisRot = smb.quaternions.q2r(thisq)
            thisTrans = tTargetInitPose + (tTargetPose - tTargetInitPose) * i
            thisPose = smb.rt2tr(thisRot, thisTrans)
            thisLegLen = self.stewart.inverseKin(thisPose)
            # result=np.row_stack([result,thisLegLen])
            # MatCurrentPose,iterc,deltaLen=mobileStewart.forwardKin(thisLegLen,lastPose,eps=1e-4, maxiter=20)
            lastPose = np.copy(thisPose)
            result = np.row_stack([result, thisLegLen])
        endt = time.process_time()
        print(endt - startt)

        return timeTraj.t, result

    def ResetController(self):
        self.framePointer = 0
        self.maxLen = len(self.buffer)

    def RunControlStep(self, col, row):
        if self.framePointer + row < self.maxLen:
            memRet = np.reshape(
                self.buffer[self.framePointer : (self.framePointer + row), :], (1, -1)
            ).tolist()[0]
            self.framePointer = self.framePointer + row
            retrow = row
            print("framePointer:", self.framePointer)
        elif self.framePointer < self.maxLen:
            memRet = np.reshape(self.buffer[self.framePointer : self.maxLen, :], (1, -1)).tolist()[0]
            retrow = self.maxLen - self.framePointer
            self.framePointer = self.maxLen
            print("last framePointer:", self.framePointer)
        else:
            memRet = np.array([])
            retrow = 0
        return memRet, (col * retrow)


# class SetPointProcessionController:
#     def __init__(self,stewart, currPose,startPose, timeInterval = 5,
#         processionAngle =  5/180*math.pi, processionAngularV = 10/180*math.pi,
#         omega=5/180*math.pi, startPhase=0, bufsize=50,tick = 0.01):

#         self.spCurrPose = currPose
#         self.spStartPose = startPose
#         self.timeInterval = timeInterval
#         self.spProcessionAngle = processionAngle
#         self.spProcessionAngularV = processionAngularV
#         self.spOmega = omega
#         self.spStartPhase = startPhase
#         self.tick = tick
#         self.bufsize = bufsize
#         self.stewart = stewart
#         self.jointLength0,_ =self.stewart.zeroLegLength()

#     def ResetController(self):
#         #find the initial posture and plan trajectory to the starting pose
#         processionPoseInTraj=np.matmul(np.matmul(smb.trotz(self.spStartPhase),smb.troty(self.spProcessionAngle)),smb.trotz(-self.spStartPhase))
#         MatStartPose = np.matmul(self.spStartPose,processionPoseInTraj)

#         timeSeries, result = self.stewart.trajectoryPlanningPtp(self.spCurrPose, MatStartPose, self.timeInterval)
#         self.buffer = result-self.jointLength0
#         self.framePointer = 0
#         self.maxLen = len(self.buffer)
#         self.state = 'PTPMoving'

#         self.history=np.copy(self.buffer)

#     def GetNextProcessionFrame(self,mode,framelen):
#         if(mode =='START'):
#             self.processionPhase = self.spStartPhase

#         result=np.empty([0,6])
#         for i in np.arange(1,framelen+1):
#             processionTick = self.processionPhase+i*self.tick*self.spProcessionAngularV
#             processionPoseInTraj=np.matmul(np.matmul(smb.trotz(processionTick),smb.troty(self.spProcessionAngle)),smb.trotz(-processionTick))
#             MatprocessionPose = np.matmul(self.spStartPose,processionPoseInTraj)
#             thisLegLen=self.stewart.inverseKin(MatprocessionPose)
#             result=np.row_stack([result,thisLegLen])

#         self.processionPhase=self.processionPhase+framelen*self.tick*self.spProcessionAngularV

#         return result-self.jointLength0

#     def RunControlStep(self, col, row):
#         if(self.state == 'PTPMoving'):
#             if(self.framePointer+row<self.maxLen):
#                 memRet = np.reshape(self.buffer[self.framePointer:(self.framePointer+row),:],(1,-1)).tolist()[0]
#                 self.framePointer=self.framePointer+row
#                 retrow = row
#                 print('framePointer:',self.framePointer)
#             elif(self.framePointer<self.maxLen):
#                 memRet = np.reshape(self.buffer[self.framePointer:self.maxLen,:],(1,-1)).tolist()[0]
#                 retrow=self.maxLen-self.framePointer
#                 self.framePointer=self.maxLen
#                 print('last framePointer:',self.framePointer)

#                 #prepare for the next frame
#                 self.buffer=self.GetNextProcessionFrame('START',row)
#                 #self.history=np.row_stack([self.history,self.buffer])
#                 self.maxLen=len(self.buffer)

#                 #transfer to procession state
#                 self.state = 'Procession'
#             else:
#                 retrow=0
#         elif(self.state == 'Procession'):
#             if(row>=self.maxLen):
#                 #remaining frame is enough for transfer
#                 memRet = np.reshape(self.buffer,(1,-1)).tolist()[0]
#                 retrow = self.maxLen
#                 #preparing for the next frame
#                 self.buffer=self.GetNextProcessionFrame('CONTINUE',row)
#                 #self.history=np.row_stack([self.history,self.buffer])
#                 self.maxLen=len(self.buffer)

#             elif(row>0):
#                 #the frame length is too long remaining frame should be left
#                 memRet = np.reshape(self.buffer[0:row,:],(1,-1)).tolist()[0]
#                 retrow=row

#                 #remaining data is obtained
#                 self.buffer=self.buffer[row:self.maxLen,:]
#                 self.maxLen=len(self.buffer)

#                 #the remaining data is not enough, get new data for the next transfer
#                 if(self.maxLen<row):
#                     self.buffer=np.row_stack([self.buffer,self.GetNextProcessionFrame('CONTINUE',row-self.maxLen)])
#                     #self.history=np.row_stack([self.history,self.buffer])

#                 self.maxLen = len(self.buffer)
#                 self.state = 'Procession'
#             else:
#                 retrow=0

#         return memRet, (col*retrow)


class MXController(trajectoryPlanner):
    def __init__(self, stewart, tgtPlatformTraj, stewart_calibration, bufsize=50, tick=0.01):

        super(MXController, self).__init__(stewart, stewart_calibration)

        sz = np.shape(tgtPlatformTraj)
        self.timeStamp = tgtPlatformTraj[:, 0]
        self.stewartPositionTraj = tgtPlatformTraj[:, 1:4]

        self.stewartPoseTraj = np.empty((0, 1), dtype="quaternion")
        for i in range(sz[0]):
            thisEuler = tgtPlatformTraj[i, 4:]
            thisq = quaternion.from_rotation_matrix(
                smb.rpy2r(thisEuler[2], thisEuler[1], thisEuler[0], order="xyz")
            )
            if thisq.w < 0:
                thisq = -thisq
            self.stewartPoseTraj = np.append(self.stewartPoseTraj, thisq)

        # procession parameter for initialization
        self.tick = tick
        self.bufsize = bufsize

        # obtain the control zero point for 6 axis
        # self.jointLength0,_ =self.stewart.zeroLegLength()

    def RunControlStep(self, col, row):
        if self.framePointer + row < self.maxLen:
            memRet = np.reshape(
                self.buffer[self.framePointer : (self.framePointer + row), :], (1, -1)
            ).tolist()[0]
            self.framePointer = self.framePointer + row
            retrow = row
            print("framePointer:", self.framePointer)
        elif self.framePointer < self.maxLen:
            memRet = np.reshape(self.buffer[self.framePointer : self.maxLen, :], (1, -1)).tolist()[0]
            retrow = self.maxLen - self.framePointer
            self.framePointer = self.maxLen
            print("last framePointer:", self.framePointer)
        else:
            memRet = np.array([])
            retrow = 0
        return memRet, (col * retrow)

    def GetInitPoseOfStewart(self):
        thisPose = self.stewartPositionTraj[0, :]
        thisRot = quaternion.as_rotation_matrix(self.stewartPoseTraj[0])
        thisMat = smb.rt2tr(thisRot, thisPose)

        return MatStartPose

    def CarculateMxTraj(self):

        # time line is stored in self.timeStamp
        # target Pose for stewart is stored in self.stewartPoseTraj

        # timeTicks=np.linspace(0,100,1000)
        timeTicks = np.arange(self.timeStamp[0], self.timeStamp[-1], 0.01)
        rst = quaternion.squad(self.stewartPoseTraj, self.timeStamp, timeTicks)
        # continuousTraj=np.array([[0,1],[2,1],[3,5],[5,5]])

        kindex = np.shape(self.stewartPositionTraj)[0]
        if kindex < 4:
            print("insufficient position line found:", kindex)
        else:
            kindex = 4

        ipo3x = spi.splrep(self.timeStamp, self.stewartPositionTraj[:, 0], k=kindex - 1)
        iy3x = spi.splev(timeTicks, ipo3x)

        ipo3y = spi.splrep(self.timeStamp, self.stewartPositionTraj[:, 1], k=kindex - 1)
        iy3y = spi.splev(timeTicks, ipo3y)

        ipo3z = spi.splrep(self.timeStamp, self.stewartPositionTraj[:, 2], k=kindex - 1)
        iy3z = spi.splev(timeTicks, ipo3z)

        iy3trans = np.column_stack((iy3x, iy3y, iy3z))
        # plt.plot(timeTicks,iy3trans[:,2],self.timeStamp,self.stewartPositionTraj[:,2]+19.8+523.5)
        # plt.plot(timeTicks,np.array([quaternion.as_rotation_vector(rst[i]) for i in range(1000)])[:,1])
        # plt.plot(self.timeStamp,np.array([quaternion.as_rotation_vector(self.stewartPoseTraj[i]) for i in range(100)]),timeTicks,np.array([quaternion.as_rotation_vector(rst[i]) for i in range(1000)])[:,1])

        self.buffer = np.empty([0, 6])
        self.history = np.empty([0, 3])

        for i in range(timeTicks.shape[0]):
            # obtain the homo matrix of the current pose
            thisRot = quaternion.as_rotation_matrix(rst[i])
            thisMat = smb.rt2tr(thisRot, iy3trans[i])
            self.history = np.row_stack([self.history, quaternion.as_rotation_vector(rst[i])])

            # plus the procession movement for the current time tick
            # processionTick = self.spStartPhase+i*self.tick*self.spProcessionAngularV
            # processionPoseInTraj=np.matmul(np.matmul(smb.trotz(processionTick),smb.troty(self.spProcessionAngle)),smb.trotz(-processionTick))
            # MatprocessionPose = np.matmul(thisMat,processionPoseInTraj)

            # inverse kinematics for leg length
            thisLegLen = self.stewart.inverseKin(thisMat)
            self.buffer = np.row_stack([self.buffer, thisLegLen - self.fixed_leg_length])

        return True

    def ResetController(self):
        # reset the counter for control step
        self.framePointer = 0
        self.maxLen = len(self.buffer)

    """
    def SplitAGVStewart(self):
        # self.timeStamp=np.linspace(0,100,100)
        # inclineAngle=np.linspace(math.pi/12,math.pi/2-math.pi/12,self.timeStamp.shape[0])+(np.random.rand(self.timeStamp.shape[0])-0.5)*math.pi/6
        # #self.stewartPoseTraj=np.array([smb.r2q(smb.roty(i)) for i in inclineAngle])
        # #self.stewartPoseTraj=np.array([np.quaternion(math.cos(i/2),0,math.sin(i/2),0) for i in inclineAngle])
        # self.stewartPoseTraj=np.array([quaternion.from_rotation_vector([0,i,0]) for i in inclineAngle])
        # self.stewartPositionTraj=np.array([self.stewart.getSuggestedPose(i) for i in inclineAngle])
        # return True

        rtraj=self.mxscript.getReferenceTraj()
        self.stewartPoseTraj = np.empty([0,1],dtype='quaternion')
        self.stewartPositionTraj = np.empty([0,3])
        self.AGVPoseTraj = np.empty([0,3])
        self.timeStamp = rtraj[:,0]
        #self.history = np.empty([0,1])
        for i in rtraj:
            thisQuater=i[4:8]/la.norm(i[4:8])
            rpytheta=smb.tr2rpy(smb.r2t(smb.q2r(thisQuater)))
            thisIncline=rpytheta[1]

            #thisIncline=math.fabs(2*math.atan2(la.norm(smb.quaternions.q2v(thisQuater)),thisQuater[0]))
            if(thisIncline>math.pi/2):
                return False
            
            self.stewartPoseTraj=np.append(self.stewartPoseTraj,quaternion.from_rotation_vector([0,thisIncline,0]))
            targetCenter=self.stewart.getSuggestedPose(thisIncline)
            #self.history = np.row_stack([self.history,thisIncline])

            #targetPoseInMx = np.matmul(smb.transl(200+targetCenter[0],0,19.8+523.5+targetCenter[2]),smb.troty(stewartIncline))
            #targetPoseInWorld=smb.rt2tr(smb.quaternions.q2r(thisQuater),i[1:4])
            self.stewartPositionTraj=np.row_stack([self.stewartPositionTraj,targetCenter])
            #thisR=smb.q2r(thisQuater)
            #headingAGV=math.atan2(thisR[1,2],thisR[0,2])
            headingAGV=rpytheta[2]
            rotMxInWorld=smb.rotz(headingAGV)
            #rotMxInWorld=np.matmul(smb.quaternions.q2r(thisQuater),smb.roty(thisIncline).T)
            targetTransInMX=np.array([200+targetCenter[0],0,19.8+521.5+targetCenter[2]])
            transMxInWorld=-np.matmul(rotMxInWorld,targetTransInMX)+i[1:4]
            
            self.AGVPoseTraj=np.row_stack([self.AGVPoseTraj,np.array([transMxInWorld[0],transMxInWorld[1],headingAGV*180.0/math.pi])])
        self.AGVPoseTraj = np.column_stack([self.timeStamp,self.AGVPoseTraj])
        return True
    """


if __name__ == "__main__":
    rospy.init_node("stewart_driver")

    # first step: initialize stewart accroding to theoriotical parameters
    bottomJoints = rospy.get_param("/calibration/bottom_joints")
    platformJoints = rospy.get_param("/calibration/platform_joints")

    stewart = stewartKin.stewartKin(bottomJoints, platformJoints)

    # import the calibration result from parameter service
    stewart_calibration = {}
    stewart_calibration["bottom_transform"] = np.array(rospy.get_param("/calibration/bottom_transform"))
    stewart_calibration["platform_transform"] = np.array(rospy.get_param("/calibration/platform_transform"))
    stewart_calibration["fixed_leg_length"] = np.array(rospy.get_param("/calibration/fixed_leg_length"))

    # obtain the basic config of the stewart, e.g. theoriotical zero point and make some changes as target pose
    basicPlanner = trajectoryPlanner(stewart, stewart_calibration)
    startConf = np.copy(basicPlanner.bottom2Platform)
    tgtConf = basicPlanner.bottom2Platform @ smb.rt2tr(
        smb.rpy2r(math.pi / 180.0 * 0.0, 0.0, 0.0), np.array([0, 0, 0.0045])
    )

    device = beckhoff_device_nofity("192.168.1.103.1.1", "192.168.1.100")
    # device.enableBeckhoffMotor(False)
    device.disableFifo()

    time.sleep(1.0)
    device.enableBeckhoffMotor()
    simuLegLength = device.readLegControlLength()

    # first of all, go to zero point
    ptpplanner = PtPController(stewart, stewart_calibration, simuLegLength, startConf, mode="Leg2Pose")
    ptpplanner.ResetController()

    device.restartFifo()
    if device.echoTrajectoryProc([ptpplanner], wait=0.0) > 0:
        time.sleep(0.2)
        device.startFifoRunning()

    while device.echoTrajectoryProc([ptpplanner]) != 0:
        pass
    time.sleep(0.5)
    device.disableFifo()

    # ptpplanner = PtPController(stewart,stewart_calibration,startConf,tgtConf)
    # ptpplanner.ResetController()

    # device.restartFifo()
    # if (device.echoTrajectoryProc([ptpplanner],wait=0.0) > 0):
    #     time.sleep(1.0)
    #     device.startFifoRunning()

    # while (device.echoTrajectoryProc([ptpplanner]) != 0):
    #     pass
    # time.sleep(0.5)
    # device.disableFifo()

    sampleTraj = []
    timeStep = 1.0
    for i in range(11):
        thisConfig = [i * timeStep]
        rpy = smb.tr2rpy(startConf, order="xyz")
        xyz = np.copy(startConf[0:3, 3])

        rpy[0] = rpy[0] + i * math.pi / 180.0 * 0.2
        xyz[2] = xyz[2] + i * 0.005
        thisConfig.extend(xyz)
        thisConfig.extend(rpy)
        sampleTraj.append(thisConfig)

    mxplanner = MXController(stewart, np.array(sampleTraj), stewart_calibration)
    mxplanner.CarculateMxTraj()
    mxplanner.ResetController()

    device.restartFifo()
    if device.echoTrajectoryProc([mxplanner], wait=0.0) > 0:
        time.sleep(0.2)
        device.startFifoRunning()

    while device.echoTrajectoryProc([mxplanner]) != 0:
        pass
    time.sleep(0.5)
    device.disableFifo()

    simuLegLength = mxplanner.buffer[-1, :]
    ptpplanner = PtPController(stewart, stewart_calibration, simuLegLength, startConf, mode="Leg2Pose")
    ptpplanner.ResetController()

    device.restartFifo()
    if device.echoTrajectoryProc([ptpplanner], wait=0.0) > 0:
        time.sleep(0.2)
        device.startFifoRunning()

    while device.echoTrajectoryProc([ptpplanner]) != 0:
        pass
    time.sleep(0.5)
    device.disableFifo()

    print("end of test, exiting...")
    time.sleep(10.0)

    device.close()
    rospy.spin()
