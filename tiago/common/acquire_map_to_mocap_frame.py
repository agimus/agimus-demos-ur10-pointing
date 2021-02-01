from __future__ import print_function
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import rospy

rospy.init_node("map_to_mocap_calibration")

import pinocchio, numpy as np

bMm = pinocchio.SE3(
        np.array([
            [  0.0017117,  -0.999995, 0.00255657,],
            [   0.999995, 0.00171833, 0.00259174,],
            [-0.00259612, 0.00255212,   0.999993,],]),
        np.array([ 0.00164797, 0.00766783, -0.00893931]))

def poseToSE3(pose):
    return pinocchio.SE3(
            pinocchio.Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z),
            np.array([ pose.position.x, pose.position.y, pose.position.z]))

# prefix
# mo: mocap
# ma: map
# b: tiago mobile base
# m: mocap frame on tiago mobile base
def acquire_measurement():
    locmsg = rospy.wait_for_message("/amcl_pose", PoseWithCovarianceStamped)
    mocmsg = rospy.wait_for_message("/qualisys/tiago_base/pose", PoseStamped)

    maMb = poseToSE3(locmsg.pose.pose)
    moMm = poseToSE3(mocmsg.pose)
    moMb = moMm * bMm.inverse()
    return maMb, moMb

def calibrate(measurements):
    """ compute maMmo """
    def err_jac(maMmo):
        err=[]
        jac=[]
        for maMb, moMb in measurements:
            M = maMb.inverse() * maMmo * moMb
            err.extend(pinocchio.log6(M).vector.tolist())
            jac.extend(np.dot(pinocchio.Jlog6(M), moMb.toActionMatrixInverse()).tolist())
        return np.array(err), np.array(jac)

    maMmo = pinocchio.SE3.Identity()
    iter=100
    ethr=0.001
    Jthr=0.001
    mthr=1e-4

    def norm2(a): return np.sum(a**2)
    from numpy.linalg import norm

    while iter > 0:
        err, J = err_jac(maMmo)
        els = norm2(err)
        if norm(err) < ethr:
            print("Error is very small")
            break
        if norm(J) < Jthr:
            print("Jacobian is very small")
            break
        d,res,rank,s = np.linalg.lstsq(J, -err)
        # do line search on els = norm2(err), Jls = 2 * err^T * J
        # els(u) = norm2(err(q + u*d)) ~ els(0) + u * Jls * d
        Jls = 2 * np.dot(err,J)
        m = np.dot(Jls, d)
        if abs(m) < mthr:
            print("m is very small.", m)
            break
        assert m < 0, str(m) + " should be negative"
        alpha = 1.
        c = 0.1 # factor for the linear part
        rate = 0.5
        alphaDefault = None
        while True:
            maMmo2 = maMmo * pinocchio.exp6(alpha*d)
            err2, _ = err_jac(maMmo2)
            els2 = norm2(err2)
            if els2 < els + c * alpha * m:
                break
            if alphaDefault is None and els2 < els:
                alphaDefault = alpha
            alpha *= rate
            if alpha < 1e-5:
                if alphaDefault is None:
                    print("failed to find a alpha that makes the error decrease. m =", m)
                    return maMmo
                print("failed to find correct alpha")
                alpha = alphaDefault
                maMmo2 = maMmo * pinocchio.exp6(alpha*d)
                break
        if iter%10 == 0:
            print("{:4} {:^8} {:^8} {:^8} {:^8}".format("iter", "err", "J", "d","alpha"))
        print("{:4} {:8.5} {:8.5} {:8.5} {:8.5}".format(iter, np.sqrt(els2), norm(J), norm(d), alpha))
        maMmo = maMmo2
        iter -= 1
    return maMmo

measurements = []
while True:
    word = raw_input("Press Enter to acquire measurement or stop when done acquiring data.")
    if word == "stop": break
    measurements.append(acquire_measurement())

maMmo = calibrate(measurements)
moMma = maMmo.inverse()
print(("MoCap frame in map frame:" + " {}"*6).format(
    *(maMmo.translation.tolist()+pinocchio.utils.matrixToRpy(maMmo.rotation)[-1::-1].tolist())))
print(("Map frame in MoCap frame:" + " {}"*6).format(
    *(moMma.translation.tolist()+pinocchio.utils.matrixToRpy(moMma.rotation)[-1::-1].tolist())))
