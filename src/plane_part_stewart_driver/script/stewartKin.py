import math

# import roboticstoolbox as rtb
import time

import numpy as np
import numpy.linalg as la
import numpy.matlib as ml
import rospy

# import spatialmath as sm
import spatialmath.base as smb


class stewartKin:
    def __init__(self, bottomJoints, platformJoints):
        self.bottomJoints = bottomJoints
        self.platformJoints = platformJoints

        self.platformJointsHom = np.column_stack([self.platformJoints, np.ones(6)])
        self.platformJointsHom = self.platformJointsHom.T

        self.bottomJointsHom = np.column_stack([self.bottomJoints, np.ones(6)])
        self.bottomJointsHom = self.bottomJointsHom.T

        print(self.platformJointsHom)
        print(self.bottomJointsHom)

        """
        #this is the basic transformation for stewart to AGV
        self.MatAgv2stewart=smb.transl(200,0,19.8)
        self.zeroTargetHeight = 521.5
        self.targetOrigin=np.array([214.7,0,417.4])
        self.zeroTargetIncline = 60*math.pi/180

        stewartFixedCircleRadius = 209.73
        stewartNeiboringAngle = 22.14 / 180.0 * math.pi
        stewartPlatformCircleRadius = 129.65
        stewartPlatformAngle = 19.42 / 180.0 * math.pi

        #self.postureCenter=np.array([-280,0,400])
        #self.postureEcllipseDiameter=np.array([180,480])

        #self.postureCenter=np.array([-300,0,440])
        #self.postureEcllipseDiameter=np.array([130,550])

        self.postureCenter=np.array([-340,0,400])
        self.postureEcllipseDiameter=np.array([180,520])

        MatTarget2Platform = np.matmul(smb.troty(-self.zeroTargetIncline),smb.transl(-self.targetOrigin[0],-self.targetOrigin[1],-self.targetOrigin[2]))

        self.stewartBaseJoints=np.array([[stewartFixedCircleRadius*math.cos(stewartNeiboringAngle/2),stewartFixedCircleRadius*math.sin(stewartNeiboringAngle/2),0],
        [stewartFixedCircleRadius*math.cos(-stewartNeiboringAngle/2+math.pi*2/3),stewartFixedCircleRadius*math.sin(-stewartNeiboringAngle/2+math.pi*2/3),0],
        [stewartFixedCircleRadius*math.cos(stewartNeiboringAngle/2+math.pi*2/3),stewartFixedCircleRadius*math.sin(stewartNeiboringAngle/2+math.pi*2/3),0],
        [stewartFixedCircleRadius*math.cos(-stewartNeiboringAngle/2-math.pi*2/3),stewartFixedCircleRadius*math.sin(-stewartNeiboringAngle/2-math.pi*2/3),0],
        [stewartFixedCircleRadius*math.cos(stewartNeiboringAngle/2-math.pi*2/3),stewartFixedCircleRadius*math.sin(stewartNeiboringAngle/2-math.pi*2/3),0],
        [stewartFixedCircleRadius*math.cos(-stewartNeiboringAngle/2),stewartFixedCircleRadius*math.sin(-stewartNeiboringAngle/2),0]])

        platformAngleList=[
            -stewartPlatformAngle/2+math.pi/3,
            stewartPlatformAngle/2+math.pi/3,
            -stewartPlatformAngle/2+math.pi,
            stewartPlatformAngle/2-math.pi,
            -stewartPlatformAngle/2-math.pi/3,
            stewartPlatformAngle/2-math.pi/3
        ]

        self.stewartPlatformJoints=np.array([
            [stewartPlatformCircleRadius*math.cos(platformAngleList[0]),stewartPlatformCircleRadius*math.sin(platformAngleList[0]),0],
            [stewartPlatformCircleRadius*math.cos(platformAngleList[1]),stewartPlatformCircleRadius*math.sin(platformAngleList[1]),0],
            [stewartPlatformCircleRadius*math.cos(platformAngleList[2]),stewartPlatformCircleRadius*math.sin(platformAngleList[2]),0],
            [stewartPlatformCircleRadius*math.cos(platformAngleList[3]),stewartPlatformCircleRadius*math.sin(platformAngleList[3]),0],
            [stewartPlatformCircleRadius*math.cos(platformAngleList[4]),stewartPlatformCircleRadius*math.sin(platformAngleList[4]),0],
            [stewartPlatformCircleRadius*math.cos(platformAngleList[5]),stewartPlatformCircleRadius*math.sin(platformAngleList[5]),0]
        ])

        self.stewartPlatformJointsHom=np.column_stack([self.stewartPlatformJoints,np.ones(6)])
        self.stewartPlatformJointsHom=self.stewartPlatformJointsHom.T

        self.stewartBaseJointsHom = np.column_stack([self.stewartBaseJoints,np.ones(6)])
        self.stewartBaseJointsHom = self.stewartBaseJointsHom.T

        print(self.stewartPlatformJoints)
        print(self.stewartBaseJoints)

        self.platformJointsInTarget = np.matmul(MatTarget2Platform,self.stewartPlatformJointsHom)
        #print(invt(MatTarget2Platform)*stewartPlatformJointsHom)

        self.baseJointsInAGV = np.matmul(self.MatAgv2stewart,self.stewartBaseJointsHom)
        return
        """

    def inverseKin(self, MatTargetPose):
        # parameters for procession movement
        platformJointsRstBottom = MatTargetPose @ self.platformJointsHom
        jointsLength = platformJointsRstBottom - self.bottomJointsHom
        # print(jointsLength)
        jointLengthAbs = la.norm(jointsLength, axis=0)
        return jointLengthAbs

    def forwardKin(self, jointLengthAbs, MatInitPose, maxiter=1000, eps=1e-6):
        x0 = np.copy(MatInitPose)
        exitCondition = False
        iterc = 0

        while not exitCondition:
            platformJointsRstBottom = x0 @ self.platformJointsHom
            jointsLength = platformJointsRstBottom - self.bottomJointsHom
            currLen = la.norm(jointsLength, axis=0)

            legVector = np.divide(jointsLength, currLen)
            legVector = legVector[0:3, :]
            # print(legVector)
            obVector = self.platformJointsHom[0:3, :]
            # print(obVector)
            jacobi1 = np.cross(obVector, legVector, axis=0)
            jacobi = np.column_stack([jacobi1.T, 1000 * legVector.T])

            invJacobi = la.inv(jacobi)
            delta = np.mat(currLen - jointLengthAbs)
            delta = delta.T
            velocityVec = -invJacobi @ delta
            rotation = np.asarray(velocityVec[0:3])
            translation = np.asarray(velocityVec[3:6])

            angle = la.norm(rotation)
            if angle == 0:
                vect = np.array([1, 0, 0])
            else:
                vect = rotation / la.norm(rotation)
            deltaRot = smb.angvec2r(angle * 0.5, vect)

            # deltaRot= rodrigusMat(rotation)
            # print(deltaRot)

            x0[0:3, 0:3] = np.matmul(deltaRot, x0[0:3, 0:3])
            x0[0:3, 3] = x0[0:3, 3] + 0.5 * 1000 * translation.T

            if la.norm(velocityVec) < 1e-12:
                exitCondition = True
            if la.norm(delta) < eps:
                exitCondition = True
            iterc = iterc + 1
            if iterc > maxiter:
                exitCondition = True
            # print(iterc,delta)
            # print(iterc,velocityVec)

        return x0, iterc, delta


if __name__ == "__main__":
    rospy.init_node("stewart_driver")
    bottomJoints = rospy.get_param("/calibration/bottom_joints")
    platformJoints = rospy.get_param("/calibration/platform_joints")
    stewart = stewartKin(bottomJoints, platformJoints)

    rospy.spin()
