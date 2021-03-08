import csv, numpy as np

def read(filename):
    with open(filename, 'r') as f:
        csv_reader = csv.reader(f, delimiter='\t')
        # headers
        for i in range(12):
            csv_reader.next()
        data = np.array([ [ float(x) for x in row[2:-1] ] for row in csv_reader ])
        return data

import matplotlib.pyplot as plt

d0 = read("tiago_calibration_0_6D.tsv")
d1 = read("tiago_calibration_1_6D.tsv")
d2 = read("tiago_calibration_2_6D.tsv")

def plot(d):
    ax=plt.subplot(3,1,1)
    ax.plot(d[:,0], d[:,1])
    ax=plt.subplot(3,1,2)
    ax.plot(d[:,2])
    ax.plot(d[:,3])
    ax.plot(d[:,4])
    ax=plt.subplot(3,1,3)
    ax.plot(d[:,5])
    plt.show()

def plotTrajectory(d, xy=True, z=False, angles=False):
    nrows = xy+z+angles
    irows = 1
    if xy:
        ax = plt.subplot(nrows,1,1)
        ax.plot([ e.translation[0] for e in d ], [ e.translation[1] for e in d ])
        ax.set_aspect('equal', adjustable='box')
        irows = 2
    if z:
        plt.subplot(nrows,1,irows)
        plt.plot([ e.translation[2] for e in d ], label="tz")
        irows += 1
    if angles:
        plt.subplot(nrows,1,irows)
        plt.plot([ pinocchio.log3(e.rotation) for e in d ])
        plt.legend(["rx", "ry", "rz"])
    plt.show()

# m = MoCap
# b = base
# w = world
#
# Odometry:
# wMb(i+1) = wMb(i) * exp(v*dt)
# wMm(i+1) * mMb = wMm(i) * mMb * exp(v*dt)
# bMm * wMm(i+1).inverse() * wMm(i) * mMb * exp(v*dt) = Id
#
# Balanced odometry drift
# 
# Global positioning
# wMm(i) = wMb(0) * exp(v * i * dt) * bMm
# Worse because of the cumulative odometry drift.

import pinocchio
def toSE3(e):
    if e[0] == 0.0: return None
    return pinocchio.SE3(
            #e[7:16].reshape((3,3)),
            pinocchio.utils.rpyToMatrix(np.deg2rad(e[3:6])),
            e[:3]/1000)
Id = pinocchio.SE3.Identity()

d0se3 = [ toSE3(e) for e in d0 ]
d1se3 = [ toSE3(e) for e in d1 ]
d2se3 = [ toSE3(e) for e in d2 ]
v0 = np.array([0. , 0., 0., 0., 0.,  0.3])
v1 = np.array([0.1, 0., 0., 0., 0., -0.3])
v2 = np.array([0.1, 0., 0., 0., 0.,  0.3])
dt = 0.01
xReg = 1.
K = 5

dMocap = []
dOdom = []
for wMm, v in [
        (d0se3, v0),
        (d1se3, v1),
        (d2se3, v2),
        ]:
    for i in range(len(wMm)-K):
        if wMm[i+K] is None or wMm[i] is None: continue
        dMocap.append(wMm[i+K].inverse() * wMm[i])
        dOdom.append(pinocchio.exp6(K*v*dt))
subsample = 10
if subsample > 1:
    dMocap = dMocap[::10]
    dOdom = dOdom[::10]

def error(dMocap, dOdom, bMm):
    # x regularisation
    if xReg > 0:
        errs = (xReg * pinocchio.log6(bMm).vector).tolist()[2:5]
    else:
        errs = []
    assert len(dMocap) == len(dOdom)
    mMb = bMm.inverse()
    for dM, dO in zip(dMocap, dOdom):
        P = bMm * dM * mMb * dO
        errs.extend(pinocchio.log6(P).vector.tolist())
    return np.array(errs)

def jacobian(dMocap, dOdom, bMm):
    if xReg > 0:
        J = (xReg * pinocchio.Jlog6(bMm))[2:5,:].tolist()
    else:
        J = []
    mMb = bMm.inverse()
    for dM, dO in zip(dMocap, dOdom):
        P = bMm * dM * mMb * dO
        JP = (dM * mMb * dO).toActionMatrixInverse() - (mMb*dO).toActionMatrixInverse()
        JlogP = np.dot(pinocchio.Jlog6(P), JP)
        J.extend(JlogP)
    return np.array(J)

def jacobian_fd(dMocap, dOdom, bMm, eps=0.001):
    dM = np.zeros((6))
    err = error(dMocap, dOdom, bMm)
    J = np.zeros( (err.shape[0], 6) )
    for i in range(6):
        dM[i] = eps
        bMm2 = bMm * pinocchio.exp6(dM)
        err2 = error(dMocap, dOdom, bMm2)
        J[:,i] = (err2-err)/eps
        dM[i] = 0.
    return J

def optimize(dMocap, dOdom,
        bMm = pinocchio.SE3.Identity(),
        iter=100,
        ethr=0.001,
        Jthr=0.001,
        mthr=1e-4,
        fd=False,
        ):
    def norm2(a): return np.sum(a**2)
    from numpy.linalg import norm
    err2 = None
    while iter > 0:
        err = error(dMocap, dOdom, bMm) if err2 is None else err2
        els = norm2(err) if err2 is None else els2
        if norm(err) < ethr:
            print("Error is very small")
            break
        if fd:
            J = jacobian_fd(dMocap, dOdom, bMm)
        else:
            J = jacobian(dMocap, dOdom, bMm)
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
            bMm2 = bMm * pinocchio.exp6(alpha*d)
            err2 = error(dMocap, dOdom, bMm2)
            els2 = norm2(err2)
            if els2 < els + c * alpha * m:
                break
            if alphaDefault is None and els2 < els:
                alphaDefault = alpha
            alpha *= rate
            if alpha < 1e-5:
                if alphaDefault is None:
                    print("failed to find a alpha that makes the error decrease. m =", m)
                    return bMm
                print("failed to find correct alpha")
                alpha = alphaDefault
                bMm2 = bMm * pinocchio.exp6(alpha*d)
                err2 = error(dMocap, dOdom, bMm2)
                els2 = norm2(err2)
                break
        if iter%10 == 0:
            print("{:4} {:^8} {:^8} {:^8} {:^8}".format("iter", "err", "J", "d","alpha"))
        print("{:4} {:8.5} {:8.5} {:8.5} {:8.5}".format(iter, np.sqrt(els2), norm(J), norm(d), alpha))
        #bMm = bMm * pinocchio.exp6(d)
        bMm = bMm2
        iter -= 1
    return bMm

def plotError(dMocap, dOdom, bMm):
    err = error(dMocap, dOdom, bMm)
    plt.subplot(2,1,1)
    plt.plot(err[0::6], label="tx")
    plt.plot(err[1::6], label="ty")
    plt.plot(err[2::6], label="tz")
    plt.legend()
    plt.subplot(2,1,2)
    plt.plot(err[3::6], label="rx")
    plt.plot(err[4::6], label="ry")
    plt.plot(err[5::6], label="rz")
    plt.legend()
    plt.show()

def plotOdom(mMocap, bMm):
    mMb = bMm.inverse()
    bMocap = [ None if mM is None else mM * mMb for mM in mMocap ]
    deltas = []
    K = 10
    for i in range(len(bMocap)-K):
        if bMocap[i+K] is None or bMocap[i] is None: continue
        deltas.append(pinocchio.log6(bMocap[i].inverse() * bMocap[i+K]).vector / (K*dt))
    deltas = np.array(deltas)
    plt.subplot(2,1,1)
    plt.plot(deltas[:, 0], label="vx")
    plt.plot(deltas[:, 1], label="vy")
    plt.plot(deltas[:, 2], label="vz")
    plt.legend()
    plt.subplot(2,1,2)
    plt.plot(deltas[:, 3], label="wx")
    plt.plot(deltas[:, 4], label="wy")
    plt.plot(deltas[:, 5], label="wz")
    plt.legend()
    plt.show()

bMm = optimize(dMocap, dOdom, Id, mthr=1e-8)
print(bMm)

if False:
    kwargs={ 'xy': True,
             'z': True,
             'angles': False,
             }
    plt.ion()
    plt.figure()
    plt.title('Traj 0')
    plotTrajectory(d0se3, **kwargs)
    plotTrajectory([ e*bMm.inverse() for e in d0se3], **kwargs)
    plt.legend(['mocap', 'calibrated'])

    plt.figure()
    plt.title('Traj 1')
    plotTrajectory(d1se3, **kwargs)
    plotTrajectory([ e*bMm.inverse() for e in d1se3], **kwargs)
    plt.legend(['mocap', 'calibrated'])

    plt.figure()
    plt.title('Traj 2')
    plotTrajectory([ e for e in d2se3 if e is not None], **kwargs)
    plotTrajectory([ e*bMm.inverse() for e in d2se3 if e is not None], **kwargs)
    plt.legend(['mocap', 'calibrated'])
