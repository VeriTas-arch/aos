import numpy as np
from tf import transformations


class matrixHelper:
    def getTransformVector(tq):
        p = np.array([tq.translation.x, tq.translation.y, tq.translation.z, 1.0])
        q = np.array([tq.rotation.x, tq.rotation.y, tq.rotation.z, tq.rotation.w])

        # if (inv):
        #     ret = transformations.quaternion_matrix(transformations.quaternion_inverse(q))
        #     invp = -ret@p
        #     ret[0:3,3] = invp[0:3]
        # else:
        #     ret = transformations.quaternion_matrix(q)
        #     ret[0:3,3] = p[0:3]
        ret = p[0:3]
        ret = np.append(ret, transformations.euler_from_quaternion(q))

        return ret

    def getRelativeTransformMatrix(ts, tt):
        qs = np.array(ts[3:])
        qt = np.array(tt[3:])
        ps = np.array(ts[0:3])
        pt = np.array(tt[0:3])
        deltap = pt - ps  # [pt[i]-ps[i] for i in range(len(ps))]

        rotation = transformations.quaternion_multiply(
            transformations.quaternion_inverse(qs), qt
        )
        trans = matrixHelper.getQuaternionTransformPoint(
            transformations.quaternion_inverse(qs), deltap
        )

        ret = transformations.quaternion_matrix(rotation)
        ret[0:3, 3] = trans[0:3]

        return ret

    def getQuaternionTransformPoint(q, p):
        purep = np.append(p, 0.0)

        ret = transformations.quaternion_multiply(
            transformations.quaternion_multiply(q, purep),
            transformations.quaternion_inverse(q),
        )
        return ret
