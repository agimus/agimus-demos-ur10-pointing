#!/usr/bin/env python2.7
from __future__ import print_function
import argparse, rospy, tf2_ros, tf, numpy as np
from geometry_msgs.msg import TransformStamped


parser = argparse.ArgumentParser()

def add_bool_option(p, opt, *args, **kwargs):
    p.add_argument("--{}".format(opt), action="store_true", *args, **kwargs)
    p.add_argument("--no-{}".format(opt), action="store_false", *args, **kwargs)

parser.add_argument("-t", "--tag", dest="tags", action="append", type=int,
        help="The ID of a tag")
parser.add_argument("-tq", "--transform-quat", dest="poses", action="append", type=float, nargs=7,
        help="The pose of a tag")
parser.add_argument("-te", "--transform-euler", dest="poses", action="append", type=float, nargs=6,
        help="The pose of a tag")
parser.add_argument("-s", "--size", dest="size", required=True, type=float,
        help="Size of the tag (excluding the white border) in meter")
parser.add_argument("--child-frame-format", dest="child_frame_fmt", default="tag36_11_{:0>5d}",
        type=str, help="Name of child frame in tf tree that will be formatted using Python str.format")
parser.add_argument("--group-frame", dest="group", required=True,
        type=str, help="Name of the group frame (a valid frame in tf tree)")
parser.add_argument("--measurement-parent-frame", dest="meas_parent", required=True,
        type=str, help="Name of measurement parent frame in tf tree")

add_bool_option(parser, "description", dest="description", default=True, help="Put robot description in ROS param")

args = parser.parse_args(rospy.myargv()[1:])

readPosesFromTf = (args.poses is None)
assert readPosesFromTf or len(args.tags) == len(args.poses), "There should be either no poses or as many poses as tags."

tag_id_fmt = "tag36_11_{:0>5d}"
param_tag_description_fmt = "/grippers/" + tag_id_fmt + "_description"

def to_numpy(transform):
    from tf import transformations as t
    M = np.eye(4);
    M[:3,3] = [ transform.translation.x, transform.translation.y, transform.translation.z, ]
    M[:3,:3]= t.quaternion_matrix([ transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w, ])[:3,:3]
    return M

def to_tf_transform(M):
    from geometry_msgs.msg import Transform
    trans = tf.transformations.translation_from_matrix(M)
    quat  = tf.transformations.quaternion_from_matrix (M)
    ret = Transform()
    ret.translation.x, ret.translation.y, ret.translation.z = trans
    ret.rotation.x, ret.rotation.y, ret.rotation.z, ret.rotation.w = quat
    return ret

def lookup_newest_transform (buf, tf_to, tf_from, stamp):
    try:
        return buf.lookup_transform (tf_to, tf_from, stamp).transform, True
    except tf2_ros.LookupException as e:
        from geometry_msgs.msg import Transform
        rospy.logwarn(e);
        return Transform(), False
    except tf2_ros.ExtrapolationException:
        return buf.lookup_transform(tf_to, tf_from, rospy.Time(0)).transform, True

class GroupOfTags(object):
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.camera_frame = rospy.get_param('cameraFrame', "rgbd_rgb_optical_frame")

    def add_urdf_params (self, args):
        import subprocess
        import rospkg
        rospack = rospkg.RosPack()
        gerard_bauzil = rospack.get_path('gerard_bauzil')

        xacro_cmd = ['xacro', '--inorder', gerard_bauzil +"/xacro/apriltag.xacro",
                "make_urdf:=true", "size:=" + str(args.size), ]

        for id in args.tags:
            tag_name = args.child_frame_fmt.format(id)
            tag_id = tag_id_fmt.format(id)
            process = subprocess.Popen(xacro_cmd + [ "name:="+tag_name, "id:="+tag_id, ],
                    stdin=None, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            out, err = process.communicate ()
            if process.returncode != 0:
                rospy.logwarn ("Failed to generate URDF: " + err)

            rospy.set_param (param_tag_description_fmt.format(id), out)

    def compute_static_transforms (self, args):
        self.pMti = {}
        self.tiMp = {}
        if readPosesFromTf:
            for id in args.tags:
                pMt = to_numpy(
                        self.tf_buffer.lookup_transform (args.group,
                            args.child_frame_fmt.format(id),
                            rospy.Time(0), rospy.Duration(1.)).transform)
                self.pMti[id] = pMt
                self.tiMp[id] = tf.transformations.inverse_matrix(pMt)
        else:
            transforms = []
            for id, pose in zip(args.tags, args.poses):

                pMt = np.eye(4);
                pMt[:3,3] = pose[:3]
                if len(pose) == 6:
                    from tf.transformations import quaternion_from_euler, euler_matrix
                    pMt[:3,:3]= euler_matrix(*reversed(pose[3:]))[:3,:3]
                elif len(pose) == 7:
                    from tf.transformations import quaternion_matrix
                    pMt[:3,:3]= quaternion_matrix(pose[3:])[:3,:3]
                else:
                    assert False, "Pose must have 6 (vector3d, RPY) or 7 (vector3d, quaternion) values."

                self.pMti[id] = pMt
                self.tiMp[id] = tf.transformations.inverse_matrix(pMt)

                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = args.group
                transform.child_frame_id = args.child_frame_fmt.format(id)
                transform.transform = to_tf_transform (pMt)
                transforms.append(transform)

            static_broadcaster = tf2_ros.StaticTransformBroadcaster()
            static_broadcaster.sendTransform(transforms)

    def add_tag_detection (self, args):
        from agimus_hpp.ros_tools import wait_for_service
        from agimus_vision.srv import AddAprilTagService
        wait_for_service("/vision/add_april_tag_detector")
        add_april_tag_detector = rospy.ServiceProxy("add_april_tag_detector", AddAprilTagService)
        for id in args.tags:
            tag_name = args.child_frame_fmt.format(id)
            add_april_tag_detector(id, args.size, tag_name, args.meas_parent)
            rospy.loginfo(
                "Added APRIL tag {0}, size {1}m, {2}, {3}".format(
                    id, args.size, tag_name, args.meas_parent
                )
            )

    def image_detection_results (self, args):
        from agimus_vision.msg import ImageDetectionResult
        rospy.Subscriber ("/agimus/vision/detection", ImageDetectionResult,
                self.handle_detection_results)

    def handle_detection_results(self, msg):
        """
        msg is of type agimus_vision.msg.ImageDetectionResult
        """
        results = [ (id, pose, residual)
                for (id, pose, residual) in zip(msg.ids, msg.poses, msg.residuals)
                if id in args.tags ]

        if len(results)>0:
            id, pose, residual = min (results, key=lambda x: x[-1])

            # p: frame relative to which measurements should be published.
            # c: camera frame
            # g(m): the (measured) group pose
            # t(m): the (measured) tag pose
            cMtm = to_numpy(pose)
            tMg = self.tiMp[id]
            cMgm = cMtm.dot(tMg)

            _pMc, ok = lookup_newest_transform (self.tf_buffer,
                    args.meas_parent, self.camera_frame, msg.header.stamp)
            pMc = to_numpy(_pMc)
            pMgm = pMc.dot(cMgm)

            transform = TransformStamped()
            transform.header.stamp = msg.header.stamp

            transform.header.frame_id = args.meas_parent
            transform.child_frame_id = args.group + "_measured"

            transform.transform = to_tf_transform (pMgm)
            self.broadcaster.sendTransform(transform)
            delay = rospy.Time.now() - msg.header.stamp
            max_delay = rospy.get_param('max_delay', 0.3)
            if delay >= rospy.Duration(max_delay):
                rospy.logwarn("Delay of transform {} is {}s"
                        .format(transform.child_frame_id, delay.to_sec()))

def run():
    rospy.init_node("group_of_tags", anonymous=True)

    group_of_tags = GroupOfTags ()

    # Step 1: add the URDFs in parameters
    if args.description:
        group_of_tags.add_urdf_params (args)

    # Step 2: Broadcast the static transformations
    group_of_tags.compute_static_transforms (args)

    # Step 3: Request tag detection from agimus_vision
    group_of_tags.add_tag_detection (args)

    # Step 4: Register to topic /agimus/vision/detection to get the ImageDetectionResult message
    group_of_tags.image_detection_results (args)

    rospy.spin()

if __name__ == "__main__":
    try:
        run()
    except rospy.ROSInterruptException:
        pass
