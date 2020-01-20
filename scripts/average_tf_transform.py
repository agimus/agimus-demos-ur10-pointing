from __future__ import print_function
import tf2_ros, rospy, argparse, pinocchio, numpy as np, eigenpy
eigenpy.switchToNumpyArray()

parser = argparse.ArgumentParser()
parser.add_argument("src", help="Source frame.")
parser.add_argument("dst", help="Destination frame.")
parser.add_argument("N", help="Number of transforms to acquire.", type=int)
parser.add_argument("--rate", help="Change frequency of TF lookup.", type=int, default=100)

args = parser.parse_args()

def get_transform(buf):
    try:
        ts = buf.lookup_transform(args.dst, args.src, rospy.Time(0), rospy.Duration(secs=1))
        ts = ts.transform
        T = pinocchio.SE3(
                eigenpy.Quaternion(ts.rotation.w, ts.rotation.x, ts.rotation.y, ts.rotation.z,),
                np.array([ts.translation.x, ts.translation.y, ts.translation.z,]),
                )
        return T
    except tf2_ros.TransformException as e:
        rospy.logwarn("Could not get transform: " + str(e))
        return None

def estimate(transforms, Tm):
    errs = [ pinocchio.log6(Tm.actInv(T)).vector for T in transforms ]
    v_mean = np.mean(errs, 0)
    v_var = np.var(errs, 0)
    Tm = Tm * pinocchio.exp6(pinocchio.Motion(v_mean))
    return Tm, v_var

def transform_msg(transforms, Tmean, Tvar):
    return """Average transform from {} transforms:
- translation: {}
- rotation (x, y, z, w): {}
- variance: {}""".format(len(transforms),
        Tmean.translation,
        pinocchio.Quaternion(Tmean.rotation).coeffs(),
        Tvar)

def run():
    rospy.init_node("pose_calibration", anonymous=True)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rate = rospy.Rate(args.rate)

    transforms = list()
    Tmean = pinocchio.SE3.Identity()
    while len(transforms) < args.N:
        if rospy.is_shutdown(): return
        T = get_transform(tf_buffer)
        if T is not None:
            transforms.append(T)
            Tmean, Tvar = estimate(transforms, Tmean)
            rospy.loginfo(transform_msg(transforms, Tmean, Tvar))
        rate.sleep()
    print(transform_msg(transforms, Tmean, Tvar))

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
