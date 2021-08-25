# {{{2 Imports and argument parsing
from __future__ import print_function
from CORBA import Any, TC_long, TC_float
from hpp.corbaserver.manipulation import Robot, loadServerPlugin, createContext, newProblem, ProblemSolver, ConstraintGraph, Rule, Constraints, CorbaClient
from hpp.gepetto.manipulation import ViewerFactory
from hpp_idl.hpp import Error as HppError
import sys, argparse, numpy as np, time, rospy
try:
    import tqdm
    def progressbar_iterable(iterable, *args, **kwargs):
        return tqdm.tqdm(iterable, *args, **kwargs)
    def progressbar_object(*args, **kwargs):
        return tqdm.tqdm(*args, **kwargs)
except ImportError:
    def progressbar_iterable(iterable, *args, **kwargs):
        return iterable
    def progressbar_object(*args, **kwargs):
        class faketqdm:
            def set_description(self, *args, **kwargs):
                pass
            def update(self, *args, **kwargs):
                pass
            def write(self, s):
                print(s)
            def close(self):
                pass
        return faketqdm()

# parse arguments
defaultContext = "corbaserver"
p = argparse.ArgumentParser (description=
                             'Initialize demo of Pyrene manipulating a box')
p.add_argument ('--context', type=str, metavar='context',
                default=defaultContext,
                help="identifier of ProblemSolver instance")
p.add_argument ('--n-random-handles', type=int, default=None,
                help="Generate a random model with N handles.")
args = p.parse_args ()
if args.context != defaultContext:
    createContext (args.context)
isSimulation = args.context == "simulation"
# 2}}}

# {{{2 Robot and problem definition

#Robot.urdfFilename = "package://tiago_data/robots/tiago_steel_without_wheels.urdf"
#Robot.srdfFilename = "package://tiago_data/srdf/tiago.srdf"
# try:
import rospy
    # Robot.urdfString = rospy.get_param('robot_description')
    # print("reading URDF from ROS param")
# except:
print("reading generic URDF")
from hpp.rostools import process_xacro, retrieve_resource
Robot.urdfString = process_xacro("package://tiago_data/robots/tiago.urdf.xacro", "robot:=steel", "end_effector:=pal-hey5", "ft_sensor:=schunk-ft", "has_second_camera_model:=true")
Robot.srdfString = ""

if args.n_random_handles is None:
    srdf_cylinder = "package://agimus_demos/srdf/cylinder.srdf"
else:
    from generate_obstacle_model import generate_srdf
    bvh_file = "/home/jmirabel/devel/hpp/src/agimus-demos/meshes/cylinder.stl"
    srdf_cylinder = "/tmp/cylinder.srdf"
    with open(srdf_cylinder, 'w') as output:
        generate_srdf(bvh_file, args.n_random_handles, output)

class Cylinder:
    urdfFilename = "package://agimus_demos/urdf/cylinder.urdf"
    srdfFilename = srdf_cylinder
    rootJointType = "freeflyer"

class Driller:
    urdfFilename = "package://gerard_bauzil/urdf/driller_with_qr_drill.urdf"
    srdfFilename = "package://gerard_bauzil/srdf/driller.srdf"
    rootJointType = "freeflyer"

class PartP72:
    urdfFilename = "package://agimus_demos/urdf/P72-with-table.urdf"
    srdfFilename = "package://agimus_demos/srdf/P72.srdf"
    rootJointType = "freeflyer"

## Reduce joint range for security
def shrinkJointRange (robot, ratio):
    for j in robot.jointNames:
        if j[:6] != "tiago/": continue
        tj = j[6:]
        if tj.startswith("torso") or tj.startswith("arm") or tj.startswith("head"):
            bounds = robot.getJointBounds (j)
            if len (bounds) == 2:
                width = bounds [1] - bounds [0]
                mean = .5 * (bounds [1] + bounds [0])
                m = mean - .5 * ratio * width
                M = mean + .5 * ratio * width
                robot.setJointBounds (j, [m, M])

print("context=" + args.context)
loadServerPlugin (args.context, "manipulation-corba.so")
newProblem()
client = CorbaClient(context=args.context)
client.basic._tools.deleteAllServants()
def wd(o):
    from hpp.corbaserver import wrap_delete
    return wrap_delete(o, client.basic._tools)
client.manipulation.problem.selectProblem (args.context)

robot = Robot("robot", "tiago", rootJointType="planar", client=client)
crobot = wd(wd(robot.hppcorba.problem.getProblem()).robot())

from tiago_fov import TiagoFOV, TiagoFOVGuiCallback
from hpp import Transform
tiago_fov = TiagoFOV(urdfString = Robot.urdfString,
        # Real field of view angles are (49.5, 60),
        fov = np.radians((60.5, 90)),
        geoms = [ "arm_3_link_0" ])
class Tag:
    def __init__(self, n, s):
        self.name = n
        self.size = s
class Tags:
    """
    Set of tags with visibility specifications

      - n_visibility_thr: minimal number of tags visible in the set to be
        able to perform an accurate localization,
      - depth_margin: increasing ratio of the pyramidal fields of view in the
        depth direction. If C is the top of the pyramid and P a vertex, P is
        moved away from C in such a way that the distance PC is multiplied by
        1 + depth_margin.
      - size_margin increasing ratio of the pyramid base.
    """
    def __init__(self, tags, n_visibility_thr, depth_margin, size_margin):
        self.tags = tags
        self.n_visibility_thr = n_visibility_thr
        self.depth_margin = depth_margin
        self.size_margin = size_margin
    @property
    def names(self):
        return [ t.name for t in self.tags ]
    @property
    def sizes(self):
        return [ t.size for t in self.tags ]
tagss = [
        Tags([ Tag('driller/tag36_11_00230', 0.0400+0.01),
               Tag('driller/tag36_11_00023', 0.0400+0.01)
               ], 
               1, 0.005, 0.1),
        Tags([ Tag('part/tag36_11_00100', 0.0600+0.01),
               Tag('part/tag36_11_00101', 0.0600+0.01),
               Tag('part/tag36_11_00102', 0.0600+0.01),
               Tag('part/tag36_11_00001', 0.0845+0.01),
               Tag('part/tag36_11_00006', 0.0845+0.01),
               Tag('part/tag36_11_00005', 0.0845+0.01),
               Tag('part/tag36_11_00015', 0.0845+0.01),
               Tag('part/tag36_11_00014', 0.0845+0.01),
               Tag('part/tag36_11_00013', 0.0845+0.01) ],
               2, 0.01, 0.1),
               ]
tiago_fov_gui = TiagoFOVGuiCallback(robot, tiago_fov, tagss)

qneutral = crobot.neutralConfiguration()
qneutral[robot.rankInConfiguration['tiago/hand_thumb_abd_joint']] = 1.5707
qneutral[robot.rankInConfiguration['tiago/hand_index_abd_joint']]  = 0.35
qneutral[robot.rankInConfiguration['tiago/hand_middle_abd_joint']] = -0.1
qneutral[robot.rankInConfiguration['tiago/hand_ring_abd_joint']]   = -0.2
qneutral[robot.rankInConfiguration['tiago/hand_little_abd_joint']] = -0.35
removedJoints = [
            'tiago/caster_back_left_1_joint',
            'tiago/caster_back_left_2_joint',
            'tiago/caster_back_right_1_joint',
            'tiago/caster_back_right_2_joint',
            'tiago/caster_front_left_1_joint',
            'tiago/caster_front_left_2_joint',
            'tiago/caster_front_right_1_joint',
            'tiago/caster_front_right_2_joint',
            'tiago/suspension_left_joint',
            'tiago/wheel_left_joint',
            'tiago/suspension_right_joint',
            'tiago/wheel_right_joint',

            # Comment this 3 joints otherwise sot is not happy
            #'tiago/hand_index_joint',
            #'tiago/hand_mrl_joint',
            #'tiago/hand_thumb_joint',

            'tiago/hand_index_abd_joint',
            'tiago/hand_index_virtual_1_joint',
            'tiago/hand_index_flex_1_joint',
            'tiago/hand_index_virtual_2_joint',
            'tiago/hand_index_flex_2_joint',
            'tiago/hand_index_virtual_3_joint',
            'tiago/hand_index_flex_3_joint',
            'tiago/hand_little_abd_joint',
            'tiago/hand_little_virtual_1_joint',
            'tiago/hand_little_flex_1_joint',
            'tiago/hand_little_virtual_2_joint',
            'tiago/hand_little_flex_2_joint',
            'tiago/hand_little_virtual_3_joint',
            'tiago/hand_little_flex_3_joint',
            'tiago/hand_middle_abd_joint',
            'tiago/hand_middle_virtual_1_joint',
            'tiago/hand_middle_flex_1_joint',
            'tiago/hand_middle_virtual_2_joint',
            'tiago/hand_middle_flex_2_joint',
            'tiago/hand_middle_virtual_3_joint',
            'tiago/hand_middle_flex_3_joint',
            'tiago/hand_ring_abd_joint',
            'tiago/hand_ring_virtual_1_joint',
            'tiago/hand_ring_flex_1_joint',
            'tiago/hand_ring_virtual_2_joint',
            'tiago/hand_ring_flex_2_joint',
            'tiago/hand_ring_virtual_3_joint',
            'tiago/hand_ring_flex_3_joint',
            'tiago/hand_thumb_abd_joint',
            'tiago/hand_thumb_virtual_1_joint',
            'tiago/hand_thumb_flex_1_joint',
            'tiago/hand_thumb_virtual_2_joint',
            'tiago/hand_thumb_flex_2_joint',
            ]
crobot.removeJoints(removedJoints, qneutral)
tiago_fov.reduceModel(removedJoints, qneutral, len_prefix=len("tiago/"))
del crobot
robot.insertRobotSRDFModel("tiago", "package://tiago_data/srdf/tiago.srdf")
robot.insertRobotSRDFModel("tiago", "package://tiago_data/srdf/pal_hey5_gripper.srdf")
robot.setJointBounds('tiago/root_joint', [-10, 10, -10, 10])
ps = ProblemSolver(robot)
ps.loadPlugin("manipulation-spline-gradient-based.so")
vf = ViewerFactory(ps)

vf.loadRobotModel (Driller, "driller")
robot.insertRobotSRDFModel("driller", "package://gerard_bauzil/srdf/qr_drill.srdf")
robot.setJointBounds('driller/root_joint', [-10, 10, -10, 10, 0, 2])
vf.loadRobotModel (PartP72, "part")
robot.setJointBounds('part/root_joint', [-2, 2, -2, 2, -2, 2])

srdf_disable_collisions_fmt = """  <disable_collisions link1="{}" link2="{}" reason=""/>\n"""
# Disable collision between tiago/hand_safety_box_0 and driller
srdf_disable_collisions = """<robot>"""
srdf_disable_collisions += srdf_disable_collisions_fmt.format("tiago/hand_safety_box", "driller/base_link")
srdf_disable_collisions += srdf_disable_collisions_fmt.format("tiago/hand_safety_box", "driller/tag_support_link_top")
srdf_disable_collisions += srdf_disable_collisions_fmt.format("tiago/hand_safety_box", "driller/tag_support_link_back")
srdf_disable_collisions += srdf_disable_collisions_fmt.format("tiago/hand_safety_box", "driller/tag_support_link_left")
srdf_disable_collisions += srdf_disable_collisions_fmt.format("tiago/hand_safety_box", "driller/tag_support_link_top_horizontal")
linka, linkb, enabled = robot.hppcorba.robot.autocollisionPairs()
for la, lb, en in zip(linka, linkb, enabled):
    if not en: continue
    disable = False
    # Disable collision between caster wheels and anything else
    if not disable:
        disable = ('caster' in la or 'caster' in lb)
    # Disable collision between tiago/hand (except hand_safety_box_0) and all other tiago links
    if not disable:
        hand_vs_other = False
        for l in [la, lb]:
            if l.startswith("tiago/hand_") and l != "tiago/hand_safety_box_0":
                disable = True
                break
    # Disable collision between driller/* and tiago/arm_[1234567]_link
    if not disable:
        for l0, l1 in [ (la, lb), (lb, la) ]:
            if l0.startswith("driller/") and l1.startswith("tiago/arm_") and l1.endswith("_link_0") and l1[10] in "1234567":
                disable = True
                break
    if disable:
        srdf_disable_collisions += srdf_disable_collisions_fmt.format(la[:la.rfind('_')], lb[:lb.rfind('_')])
# TODO
srdf_disable_collisions += "</robot>"
robot.client.manipulation.robot.insertRobotSRDFModelFromString("", srdf_disable_collisions)

vf.loadObstacleModel ("package://gerard_bauzil/urdf/gerard_bauzil.urdf", "room")
#vf.loadObstacleModel ("package://agimus_demos/urdf/P72-table.urdf", "table")
# Display Tiago Field of view.
#vf.guiRequest.append( (tiago_fov.loadInGui, {'self':None}))
# Display visibility cones.
vf.addCallback(tiago_fov_gui)

try:
    v = vf.createViewer()
except:
    print("Did not find viewer")

joint_bounds = {}
def setRobotJointBounds(which):
    for jn, bound in joint_bounds[which]:
        robot.setJointBounds(jn, bound)

joint_bounds["default"] = [ (jn, robot.getJointBounds(jn)) for jn in robot.jointNames ]
shrinkJointRange(robot, 0.6)
joint_bounds["grasp-generation"] = [ (jn, robot.getJointBounds(jn)) for jn in robot.jointNames ]
setRobotJointBounds("default")
shrinkJointRange(robot, 0.95)
joint_bounds["planning"] = [ (jn, robot.getJointBounds(jn)) for jn in robot.jointNames ]

ps.selectPathValidation("Graph-Discretized", 0.05)
#ps.selectPathValidation("Graph-Dichotomy", 0)
#ps.selectPathValidation("Graph-Progressive", 0.02)
ps.selectPathProjector("Progressive", 0.2)
ps.addPathOptimizer("EnforceTransitionSemantic")
ps.addPathOptimizer("SimpleTimeParameterization")
if isSimulation:
    ps.setMaxIterProjection (1)

ps.setParameter("SimpleTimeParameterization/safety", 0.25)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 1.0)
ps.setParameter("ManipulationPlanner/extendStep", 0.7)
ps.setParameter("SteeringMethod/Carlike/turningRadius", 0.05)

q0 = robot.getCurrentConfig()
q0[:4] = [-3., 2., 0, -1]
#q0[robot.rankInConfiguration['tiago/torso_lift_joint']] = 0.15
q0[robot.rankInConfiguration['tiago/torso_lift_joint']] = 0.34
q0[robot.rankInConfiguration['tiago/arm_1_joint']] = 0.10
q0[robot.rankInConfiguration['tiago/arm_2_joint']] = -1.47
q0[robot.rankInConfiguration['tiago/arm_3_joint']] = -0.16
q0[robot.rankInConfiguration['tiago/arm_4_joint']] = 1.87
q0[robot.rankInConfiguration['tiago/arm_5_joint']] = -1.57
q0[robot.rankInConfiguration['tiago/arm_6_joint']] = 1.3
q0[robot.rankInConfiguration['tiago/arm_7_joint']] = 0.00

#q0[robot.rankInConfiguration['part/root_joint']:] = [0.926, -0.024, 0.000,0,0,0,1]
q0[robot.rankInConfiguration['part/root_joint']:] = [0,0.3,0.8,0,0,0,1]
# 2}}}

# {{{2 Constraint graph initialization

# {{{3 Constraint definition
def lockJoint(jname, q, cname=None, constantRhs=True):
    if cname is None:
        cname = jname
    s = robot.rankInConfiguration[jname]
    e = s+robot.getJointConfigSize(jname)
    ps.createLockedJoint(cname, jname, q[s:e])
    ps.setConstantRightHandSide(cname, constantRhs)
    return cname

# Add an alignment constraint between pregrasp and grasp for each
# part handle
# \param ps ProblemSolver instance,
# \param handle name of the handle: should be "part/handle_i" where i is an
#        integer,
# \param graph constraint graph
def addAlignmentConstrainttoEdge(ps, handle, graph):
    #recover id of handle
    handleId = all_handles.index(handle)
    J1, gripperPose = ps.robot.getGripperPositionInJoint(tool_gripper)
    J2, handlePose = ps.robot.getHandlePositionInJoint(handle)
    T1 = Transform(gripperPose)
    T2 = Transform(handlePose)
    constraintName = handle + '/alignment'
    ps.client.basic.problem.createTransformationConstraint2\
        (constraintName, J1, J2, T1.toTuple(), T2.toTuple(),
         [False, True, True, False, False, False])
    # Set constraint
    edgeName = tool_gripper + ' > ' + handle + ' | 0-0_12'
    graph.addConstraints(edge = edgeName, constraints = \
                         Constraints(numConstraints=[constraintName]))
    edgeName = tool_gripper + ' < ' + handle + ' | 0-0:1-{}_21'.format(handleId)
    graph.addConstraints(edge = edgeName, constraints = \
                         Constraints(numConstraints=[constraintName]))

ljs = list()
ps.createLockedJoint("tiago_base", "tiago/root_joint", [0,0,1,0])

lockJoint("part/root_joint", q0, "lock_part", constantRhs=False)
c_lock_part = ps.hppcorba.problem.getConstraint("lock_part")

for n in robot.jointNames:
    if n.startswith('tiago/gripper_') or n.startswith('tiago/hand_'):
        ljs.append(lockJoint(n, q0))

lock_arm = [ lockJoint(n, q0) for n in robot.jointNames
        if n.startswith("tiago/arm") or n.startswith('tiago/torso')]
lock_head = [ lockJoint(n, q0) for n in robot.jointNames
        if n.startswith("tiago/head")]

# Create "Look at gripper" constraints: for (X,Y,Z) the position of the gripper in the camera frame,
# (X, Y) = 0 and Z >= 0
tool_gripper = "driller/drill_tip"
ps.createPositionConstraint("look_at_gripper", "tiago/camera_color_optical_frame", tool_gripper,
        (0,0,0), (0,0,0), (True,True,True))
look_at_gripper = ps.hppcorba.problem.getConstraint("look_at_gripper")
import hpp_idl
look_at_gripper.setComparisonType([hpp_idl.hpp.EqualToZero,hpp_idl.hpp.EqualToZero,hpp_idl.hpp.Superior])

# Create "Look at part" constraint
ps.createPositionConstraint("look_at_part", "tiago/camera_color_optical_frame", "part/to_tag_00001",
        (0,0,0), (0,0,0), (True,True,False))
look_at_part = ps.hppcorba.problem.getConstraint("look_at_part")
# 3}}}

# {{{3 Constraint graph instanciation
from hpp.corbaserver.manipulation import ConstraintGraphFactory
graph = ConstraintGraph(robot, 'graph')
factory = ConstraintGraphFactory(graph)
factory.setGrippers([ "tiago/gripper", "driller/drill_tip", ])

all_handles = ps.getAvailable('handle')
part_handles = filter(lambda x: x.startswith("part/"), all_handles)
factory.setObjects([ "driller", "part", ], [ [ "driller/handle", ], part_handles, ], [ [], [] ])

factory.setRules([
    # Forbid driller to grasp itself.
    Rule([ "driller/drill_tip", ], [ "driller/handle", ], False),
    # Tiago always hold the gripper.
    Rule([ "tiago/gripper", ], [ "", ], False), Rule([ "tiago/gripper", ], [ "part/.*", ], False),
    # Allow to associate drill_tip with part holes only.
    Rule([ "tiago/gripper", "driller/drill_tip", ], [ "driller/handle", ".*", ], True), ])
factory.generate()

graph.addConstraints(graph=True, constraints=Constraints(numConstraints=ljs))
free = "tiago/gripper grasps driller/handle"
loop_free = 'Loop | 0-0'
for n in graph.nodes.keys():
    if n == free: continue
    graph.addConstraints(node=n, constraints=Constraints(numConstraints=["look_at_gripper"]))
for e in graph.edges.keys():
    graph.addConstraints(edge=e, constraints=Constraints(numConstraints=["tiago_base"]))
#Add alignment constraints
for handle in part_handles:
    addAlignmentConstrainttoEdge(ps, handle, graph)

graph.createNode('home', priority=1000)
graph.createEdge('home', 'home', 'move_base')
graph.createEdge('home',  free , 'start_arm', isInNode="home")
graph.createEdge( free , 'home', 'end_arm', isInNode=free)

graph.addConstraints(node="home", constraints=Constraints(numConstraints=lock_arm+lock_head))
graph.addConstraints(edge="end_arm", constraints=Constraints(numConstraints=["tiago_base", "lock_part"]))
graph.addConstraints(edge="move_base", constraints=Constraints(numConstraints=['tiago/gripper grasps driller/handle', "lock_part"]))
graph.addConstraints(edge="start_arm", constraints=Constraints(numConstraints=['tiago/gripper grasps driller/handle', "lock_part"]))

cproblem = ps.hppcorba.problem.getProblem()

cgraph = cproblem.getConstraintGraph()

graph.initialize()
# 3}}}

res, q0, err = graph.generateTargetConfig('start_arm', q0, q0)
assert (res)

# {{{3 Constraint graph validation
graphValidation = ps.client.manipulation.problem.createGraphValidation()
graphValidation.validate(cgraph)
if graphValidation.hasErrors():
    print(graphValidation.str())
    print("Graph has infeasibilities")
    sys.exit(1)
elif graphValidation.hasWarnings():
    print(graphValidation.str())
    print("Graph has only warnings")
else:
    print("Graph *seems* valid.")
# 3}}}
# 2}}}

# {{{2 Functions for handling the TSP
def generate_valid_config_for_handle(handle, qinit, qguesses = [], NrandomConfig=10):
    edge = tool_gripper + " > " + handle
    ok = False
    from itertools import chain
    def project_and_validate(e, qrhs, q):
        res, qres, err = graph.generateTargetConfig (e, qrhs, q)
        return res and not tiago_fov.clogged(qres, robot, tagss) and robot.configIsValid(qres), qres
    qpg, qg = None, None
    for qrand in chain(qguesses, ( robot.shootRandomConfig() for _ in range(NrandomConfig) )):
        res, qpg = project_and_validate (edge+" | 0-0_01", qinit, qrand)
        if res:
            ok, qg = project_and_validate(edge+" | 0-0_12", qpg, qpg)
            if ok: break
    return ok, qpg, qg

def generate_valid_config(constraint, qguesses = [], NrandomConfig=10):
    from itertools import chain
    for qrand in chain(qguesses, ( robot.shootRandomConfig() for _ in range(NrandomConfig) )):
        res, qres = constraint.apply (qrand)
        if res and not tiago_fov.clogged(qres, robot, tagss) and robot.configIsValid(qres):
            return True, qres
    return False, None

class ClusterComputation:
    def __init__(self, cgraph, lock_part):
        self._cgraph = cgraph
        self._lock_part = lock_part

    ## Return constraints of node where the tool is in front of handle
    #  \param hi handle the tool should get in front of
    #  \return solver with constraints of the pregrasp node and with
    #          constraint of fixed part pose.
    #  \note the returned solver contains the following constraints
    #    \li driller in gripper,
    #    \li tool handle in pregrasp in front of hi,
    #    \li fixed position of object (parameterizable).
    #    \li camera looks at gripper
    def freeBaseConstraint(self, hi):
        ni = tool_gripper + " > " + hi + " | 0-0_pregrasp"
        cnode = wd(self._cgraph.get(graph.nodes[ni]))
        c = wd(cnode.configConstraint().getConfigProjector()).copy()
        c.add(self._lock_part, 0)
        return c

    ## Return target constraints of node where the tool is in front of handle
    #  \param hi handle the tool should get in front of
    #  \return solver with target constraints of the edge leading to the goal
    #          node.
    #  \note the returned solver contains the following constraints
    #    \li driller in gripper,
    #    \li tool handle in pregrasp in front of hi,
    #    \li fixed position of the base (parameterizable),
    #    \li fixed position of object (parameterizable).
    #    \li camera looks at gripper
    def fixedBaseConstraint(self, hi):
        ei = tool_gripper + " > " + hi + " | 0-0_01"
        cedge = wd(self._cgraph.get(graph.edges[ei]))
        return wd(cedge.targetConstraint().getConfigProjector())

    ## Return constraint of edge between pregrasp and grasp of tool gripper
    #  \param hi handle the tool should reach
    #  \return solver with target constraints of edge leading from pregrasp to
    #          grasp.
    #  \note the returned solver contains the following constraints
    #    \li driller in gripper,
    #    \li tool gripper at handle
    #    \li complement of tool gripper at handle (parameterizable)
    #    \li fixed position of the base (parameterizable),
    #    \li fixed position of object (parameterizable).
    #    \li camera looks at gripper
    def pregraspToGraspConstraint(self, hi):
        ei = tool_gripper + " > " + hi + " | 0-0_12"
        cedge = wd(self._cgraph.get(graph.edges[ei]))
        return wd(cedge.targetConstraint().getConfigProjector())

    ## Find a cluster of handles that contains one handle
    # \param hi handle contained in the cluster,
    # \param handles superset of the resulting cluster
    # \param qrhs configuration of the robot
    # \param fixed_based whether the base is fixed or can move in each cluster
    def find_cluster(self, hi, handles, qrhs, fixed_base=False, pbar=None,
            N_find_first=20,
            N_find_others=20):
        if pbar is None:
            write = print
        else:
            write = pbar.write
        #ni = part_gripper + " grasps " + hi
        #ni = part_gripper + " > " + hi + " | 0-0_pregrasp"
        qrand = q0
        best_cluster = []
        if fixed_base:
            cpi = self.fixedBaseConstraint(hi)
        else:
            cpi = self.freeBaseConstraint(hi)
        cpi.setRightHandSideFromConfig(qrhs)
        ci = self.pregraspToGraspConstraint(hi)

        step = [ False, ] * 4
        for k in range(N_find_first):
            # Compute config where Tiago is not in collision
            if k == 0:
                res, qphi = generate_valid_config(cpi, qguesses=[q0,], NrandomConfig = 0)
            else:
                res, qphi = generate_valid_config(cpi, qguesses=[], NrandomConfig = 1)
            if not res: continue
            step[0] = True
            ci.setRightHandSideFromConfig(qphi)
            res, qhi = generate_valid_config(ci, qguesses=[qphi,], NrandomConfig = 0)
            if not res: continue
            step[1] = True

            t_base = np.array(robot.hppcorba.robot.getJointsPosition(qhi, ['tiago/torso_fixed_joint'])[0][:3])
            cur_cluster = [ (hi, qphi, qhi) ]
            for hj in handles:
                if hj == hi: continue
                t_handle = np.array(robot.getHandlePositionInJoint(hi)[1][:3])
                if False and np.linalg.norm(t_handle-t_base) > 0.6:
                    #print("prune")
                    continue
                ok, qphj, qhj = generate_valid_config_for_handle(hj, qhi,
                        qguesses = [ qq for _, qq, qqq in cur_cluster ],
                        NrandomConfig=N_find_others)
                if ok:
                    cur_cluster.append((hj, qphj, qhj))
            if len(cur_cluster) > len(best_cluster):
                best_cluster = cur_cluster
                step[2] = True
        if   not step[0]: write("not able to generate pregrasp config")
        elif not step[1]: write("not able to generate grasp config")
        return best_cluster

    ## Compute clusters of handles reachable by separate base positions
    # \param handles list of handles to reach,
    # \param qrhs initial configuration of the robot.
    # \return list of clusters. Each cluster is a list of triples
    #         (handle_name, pregrasp config, config)
    def find_clusters(self, handles, qrhs,
            N_find_first=20,
            N_find_others=20):
        from random import choice
        start = time.time()
        clusters = []
        remaining_handles = handles[:]
        pbar = progressbar_object(desc="Looking for clusters", total=len(remaining_handles))
        n_consecutive_failure = 0
        while remaining_handles:
            cluster = self.find_cluster(choice(remaining_handles), remaining_handles, qrhs,
                    pbar=pbar, N_find_first=N_find_first, N_find_others=N_find_others)
            if len(cluster) == 0:
                n_consecutive_failure += 1
                if n_consecutive_failure <= 20: continue
                pbar.close()
                for h in remaining_handles:
                    print("Could not reach handle", h)
                break
            for hi, qpi, qi in cluster:
                remaining_handles.remove(hi)
            clusters.append(cluster)
            pbar.set_description(desc=str(len(clusters)) + " clusters found", refresh=False)
            pbar.update(len(cluster))
        return clusters

    ## Solve traveling salesman problem between configurations of a cluster
    # \param armPlanner motion planner for the arm only,
    # \param cluster the set of configurations to order,
    # \param qhome initial and final configuration. If not provided, the
    #        configuration with the same base pose as the first config in the
    #        cluster is computed.
    #
    # \li Build back and forth path of tool between pregrasp and grasp for each
    #     handle,
    # \li call InStatePlanner::solveTSP to compute the handle order, and
    # \li build the resulting path by concatenation.
    def solveTSP(self, armPlanner, cluster, qhome = None, pb_kwargs = {}):
        # Create home position
        if qhome is None:
            res, qhome, err = graph.generateTargetConfig('end_arm', cluster[0][1], cluster[0][1])

        configs = [ qhome ]
        # build back and forth path of tool between pregrasp and grasp for each
        # handle
        grasp_paths = []
        #print(permutation, arm_paths)
        for hi, qphi, qhi in cluster:
            # Compute grasp config
            ei = tool_gripper + " > " + hi + " | 0-0_12"
            cedge = wd(self._cgraph.get(graph.edges[ei]))
            p01 = cedge.getSteeringMethod().call(qphi, qhi)
            # from grasp to pregrasp
            eir = tool_gripper + " < " + hi + " | 0-0:1-{}_21".format(all_handles.index(hi))
            cedge = wd(self._cgraph.get(graph.edges[eir]))
            p10 = cedge.getSteeringMethod().call(qhi, qphi)

            grasp_paths.append((p01, p10))
            configs.append(qphi)

        if len(configs) == 1: return []
        permutation, arm_paths = armPlanner.solveTSP(configs, resetRoadmapEachTime=True,
                pb_kwargs = pb_kwargs)
        paths = []
        #print(permutation, arm_paths)
        for i, p in zip(permutation[1:], arm_paths):
            # p: path to go to
            # - pregrasp of cluster[i][0] if i > 0
            # - home if i == 0
            paths.append(p)
            if i > 0:
                paths.append(grasp_paths[i-1][0])
                paths.append(grasp_paths[i-1][1])
        return paths

class InStatePlanner:
    # Default path planner
    parameters = {'kPRM*/numberOfNodes': Any(TC_long,2000)}

    def __init__(self):
        self.plannerType = "BiRRT*"
        self.optimizerTypes = []
        self.maxIterPathPlanning = None
        self.timeOutPathPlanning = None

        self.manipulationProblem = wd(ps.hppcorba.problem.getProblem())
        self.crobot = self.manipulationProblem.robot()
        # create a new problem with the robot
        self.cproblem = wd(ps.hppcorba.problem.createProblem(self.crobot))
        # Set parameters
        for k, v in self.parameters.items():
            self.cproblem.setParameter(k, v)
        # Set config validation
        self.cproblem.clearConfigValidations()
        for cfgval in [ "CollisionValidation", "JointBoundValidation"]:
            self.cproblem.addConfigValidation(wd(ps.hppcorba.problem.createConfigValidation(cfgval, self.crobot)))
        # get reference to constraint graph
        self.cgraph = self.manipulationProblem.getConstraintGraph()
        # Add obstacles to new problem
        for obs in ps.getObstacleNames(True,False):
            self.cproblem.addObstacle(wd(ps.hppcorba.problem.getObstacle(obs)))

    def setEdge(self, edge):
        # Get constraint of edge
        edgeLoopFree = wd(self.cgraph.get(graph.edges[edge]))
        self.cconstraints = wd(edgeLoopFree.pathConstraint())
        self.cproblem.setPathValidation(edgeLoopFree.getPathValidation())
        self.cproblem.setConstraints(self.cconstraints)
        self.cproblem.setSteeringMethod(wd(edgeLoopFree.getSteeringMethod()))
        self.cproblem.filterCollisionPairs()

    def setReedsAndSheppSteeringMethod(self):
        sm = wd(ps.hppcorba.problem.createSteeringMethod("ReedsShepp", self.cproblem))
        self.cproblem.setSteeringMethod(sm)
        dist = ps.hppcorba.problem.createDistance("ReedsShepp", self.cproblem)
        self.cproblem.setDistance(dist)

    def buildRoadmap(self, qinit):
        wd(self.cconstraints.getConfigProjector()).setRightHandSideFromConfig(qinit)
        self.cproblem.setInitConfig(qinit)
        self.cproblem.resetGoalConfigs()
        self.roadmap = wd(ps.client.manipulation.problem.createRoadmap(
                wd(self.cproblem.getDistance()),
                self.crobot))
        self.roadmap.constraintGraph(self.cgraph)
        self.planner = wd(ps.hppcorba.problem.createPathPlanner(
            self.plannerType,
            self.cproblem,
            self.roadmap))
        if self.maxIterPathPlanning:
            self.planner.maxIterations(self.maxIterPathPlanning)
        if self.timeOutPathPlanning:
            self.planner.timeOut(self.timeOutPathPlanning)
        path = wd(self.planner.solve())
        
    def createEmptyRoadmap(self):
        self.roadmap = wd(ps.hppcorba.problem.createRoadmap(
                wd(self.cproblem.getDistance()),
                self.crobot))
        
    def computePath(self, qinit, qgoal, resetRoadmap = False):
        wd(self.cconstraints.getConfigProjector()).setRightHandSideFromConfig(qinit)
        res, qgoal2 = self.cconstraints.apply(qgoal)
        self.cproblem.setInitConfig(qinit)
        self.cproblem.resetGoalConfigs()
        self.cproblem.addGoalConfig(qgoal2)

        if resetRoadmap or not hasattr(self, 'roadmap'):
            self.createEmptyRoadmap()
        self.planner = wd(ps.hppcorba.problem.createPathPlanner(
            self.plannerType,
            self.cproblem,
            self.roadmap))
        if self.maxIterPathPlanning:
            self.planner.maxIterations(self.maxIterPathPlanning)
        if self.timeOutPathPlanning:
            self.planner.timeOut(self.timeOutPathPlanning)
        if self.maxIterPathPlanning is None and self.timeOutPathPlanning is None:
            self.planner.stopWhenProblemIsSolved(True)
        path = wd(self.planner.solve())
        for optType in self.optimizerTypes:
            optimizer = wd(ps.hppcorba.problem.createPathOptimizer(optType, self.manipulationProblem))
            try:
                optpath = optimizer.optimize(path)
                # optimize can return path if it couldn't find a better one.
                # In this case, we have too different client refering to the same servant.
                # thus the following code deletes the old client, which triggers deletion of
                # the servant and the new path points to nothing...
                # path = wd(optimizer.optimize(path))
                from hpp.corbaserver.tools import equals
                if not equals(path, optpath):
                    path = wd(optpath)
            except HppError as e:
                print("could not optimize", e)
        return path

    def timeParameterization(self, path):
        for optType in [ "EnforceTransitionSemantic", "SimpleTimeParameterization" ]:
            optimizer = wd(ps.hppcorba.problem.createPathOptimizer(optType, self.manipulationProblem))
            try:
                optpath = optimizer.optimize(path)
                # optimize can return path if it couldn't find a better one.
                # In this case, we have too different client refering to the same servant.
                # thus the following code deletes the old client, which triggers deletion of
                # the servant and the new path points to nothing...
                # path = wd(optimizer.optimize(path))
                from hpp.corbaserver.tools import equals
                if not equals(path, optpath):
                    path = wd(optpath)
            except HppError as e:
                print("could not time parameterize", e)
        return path

    ## Solve traveling salesman problem between configurations
    # \param configs list of configurations.
    #
    # \li Compute matrix of paths between all configurations by calling method
    #     computePath,
    # \li compute matrix of distances as path lengths,
    # \li call TSP solver in module agimus_demos.pytsp
    def solveTSP(self, configs, resetRoadmapEachTime, pb_kwargs={}):
        # Compute matrix of paths
        l = len(configs)
        paths = [ [ None, ] * l for _ in range(l) ]
        distances = np.zeros((l,l))
        from itertools import combinations
        pbar = progressbar_object(desc="Computing distance matrix", total=l*(l-1)/2, **pb_kwargs)
        for i, j in combinations(range(l), 2):
            try:
                path = paths[i][j] = self.computePath(configs[i],configs[j],resetRoadmap=resetRoadmapEachTime)
                distances[j,i] = distances[i,j] = path.length()
            except Exception as e:
                pbar.write("Failed to connect {} to {}: {}".format(i, j,e))
                paths[i][j] = None
                distances[j,i] = distances[i,j] = 1e8
            pbar.update()
        if l > 15:
            from agimus_demos.pytsp.approximative_kopt import solve_3opt as solve_tsp
        else:
            from agimus_demos.pytsp.dynamic_programming import solve_with_heuristic as solve_tsp
        distance, permutation = solve_tsp(distances)
        # rotate permutation so that 0 is the first index.
        solution = []
        permutation = [0,] + permutation + [0,]
        assert permutation[0] == 0, str(permutation[0]) + " should be 0"
        for i,j in zip(permutation,permutation[1:]):
            p = paths[i][j] if i < j else wd(paths[j][i].reverse())
            solution.append(p)
        return permutation, solution

    def writeRoadmap(self, filename):
        ps.client.manipulation.problem.writeRoadmap\
                       (filename, self.roadmap, self.crobot, self.cgraph)

    def readRoadmap(self, filename):
        self.roadmap = ps.client.manipulation.problem.readRoadmap\
                       (filename, self.crobot, self.cgraph)

def concatenate_paths(paths):
    if len(paths) == 0: return None
    p = paths[0].asVector()
    for q in paths[1:]:
        p.appendPath(q)
    return p
# 2}}}

# {{{2 Problem resolution

# {{{3 Finalize FOV filter
res, q, err = graph.applyNodeConstraints(free, q0)
assert res
robot.setCurrentConfig(q)
oMh, oMd = robot.hppcorba.robot.getJointsPosition(q, ["tiago/hand_tool_link", "driller/base_link"])
tiago_fov.appendUrdfModel(Driller.urdfFilename, "hand_tool_link",
        (Transform(oMh).inverse() * Transform(oMd)).toTuple(),
        prefix="driller/")
# 3}}}

# {{{3 Create InStatePlanner
armPlanner = InStatePlanner ()
armPlanner.setEdge(loop_free)
#armPlanner.optimizerTypes = [ "SplineGradientBased_bezier3", ]
armPlanner.optimizerTypes = [ ]
armPlanner.cproblem.setParameter("SimpleTimeParameterization/safety", Any(TC_float, 0.25))
armPlanner.cproblem.setParameter("SimpleTimeParameterization/order", Any(TC_long, 2))
armPlanner.cproblem.setParameter("SimpleTimeParameterization/maxAcceleration", Any(TC_float, 1.0))
armPlanner.maxIterPathPlanning = 600
armPlanner.timeOutPathPlanning = 10.
# Set collision margin between mobile base and the rest because the collision model is not correct.
bodies = ("tiago/torso_fixed_link_0", "tiago/base_link_0")
cfgVal = wd(armPlanner.cproblem.getConfigValidations())
pathVal = wd(armPlanner.cproblem.getPathValidation())
for _, la, lb, _, _ in zip(*robot.distancesToCollision()):
    if la in bodies or lb in bodies:
        cfgVal.setSecurityMarginBetweenBodies(la, lb, 0.07)
        pathVal.setSecurityMarginBetweenBodies(la, lb, 0.07)
del cfgVal
del pathVal

basePlanner = InStatePlanner ()
basePlanner.plannerType = "kPRM*"
basePlanner.optimizerTypes.append("RandomShortcut")
basePlanner.setEdge("move_base")
basePlanner.setReedsAndSheppSteeringMethod()

import security_margins
smBase = security_margins.SecurityMargins(robot.jointNames)
margin = 0.1
for i in [ 0, smBase.jid("part/root_joint") ]:
    smBase.margins[i,:] = margin
    smBase.margins[:,i] = margin
basePlanner.cproblem.setSecurityMargins(smBase.margins.tolist())

basePlanner.createEmptyRoadmap()
# 3}}}

# {{{3 Load (or create) mobile base roadmap
basePlannerUsePrecomputedRoadmap = False
if basePlannerUsePrecomputedRoadmap:
    from os import getcwd, path
    roadmap_file = getcwd() + "/roadmap-hpp.bin"
    if path.exists(roadmap_file):
        print("Reading mobile base roadmap", roadmap_file)
        basePlanner.readRoadmap(roadmap_file)
    else:
        print("Building mobile base roadmap")
        try:
            basePlanner.cproblem.setParameter('kPRM*/numberOfNodes', Any(TC_long,2000))
            basePlanner.buildRoadmap(q0)
            #sm.margins[:,:] = 0.
            #basePlanner.cproblem.setSecurityMargins(sm.margins.tolist())
        except HppError as e:
            print(e)
        print("Writing mobile base roadmap", roadmap_file)
        basePlanner.writeRoadmap(roadmap_file)

basePlanner.plannerType = "DiffusingPlanner"
basePlanner.maxIterPathPlanning = 1000
# 3}}}

# {{{3 Find handle clusters and solve TSP for each clusters.
clusters_comp = ClusterComputation(armPlanner.cgraph, c_lock_part)
setRobotJointBounds("grasp-generation")
clusters = clusters_comp.find_clusters(
        #part_handles[:4],
        part_handles,
        q0, N_find_first = 40, N_find_others = 40)
setRobotJointBounds("planning")
solve_tsp_problems = False
if solve_tsp_problems:
    clusters_path = []
    qhomes = [q0, ]
    for cluster in progressbar_iterable(clusters, "Find path for each cluster"):
        paths = clusters_comp.solveTSP(armPlanner, cluster, pb_kwargs={'position': 1})

        clusters_path.append(concatenate_paths(paths))
        if len(paths) > 0:
            qhomes.append(paths[0].initial())
# 3}}}

# {{{3 Solve TSP for the mobile base
if solve_tsp_problems:
    base_order, base_paths = basePlanner.solveTSP(qhomes, resetRoadmapEachTime=not basePlannerUsePrecomputedRoadmap)
# }}}

# 2}}}

# {{{2 Function for online reso 
def recompute_clusters(handles = None, qcurrent = None):
    if qcurrent is None:
        import estimation 
        qcurrent = estimation.get_current_config(robot, graph, q0)
        try:
            qcurrent = estimation.get_cylinder_pose(robot, qcurrent, timeout=0.5)
        except RuntimeError:
            pass
    if handles is None:
        handles = part_handles

    setRobotJointBounds("grasp-generation")
    clusters = clusters_comp.find_clusters(handles,
            qcurrent, N_find_first = 40, N_find_others = 40)
    setRobotJointBounds("planning")
    return clusters

def compute_base_path_to_cluster_init(i_cluster, qcurrent = None):
    if qcurrent is None:
        import estimation 
        qcurrent = estimation.get_current_config(robot, graph, q0)
        try:
            qcurrent = estimation.get_cylinder_pose(robot, qcurrent, timeout=0.5)
        except RuntimeError:
            pass

    setRobotJointBounds("default")
    valid, msg = robot.isConfigValid(qcurrent)
    outOfCollisionPath = None
    if not valid:
        if msg == 'Collision between object tiago/torso_fixed_column_link_0 and tiago/hand_safety_box_0' \
            or msg == 'Collision between object tiago/torso_fixed_column_link_0 and driller/base_link_0' \
            or msg == 'Collision between object tiago/base_link_0 and driller/tag_support_link_top_0' \
            or msg == 'Collision between object tiago/torso_fixed_column_link_0 and driller/tag_support_link_back_0':
            qcurrent2 = qcurrent[:]
            qcurrent2[robot.rankInConfiguration['tiago/arm_6_joint']] = 1.
            res, qcurrent2, err = graph.applyNodeConstraints("tiago/gripper grasps driller/handle", qcurrent2)
            if not res:
                print("could not get out of collision")
            else:
                csm = armPlanner.cproblem.getSteeringMethod()
                outOfCollisionPath = csm.call(qcurrent, qcurrent2)
                qcurrent = qcurrent2
        else:
            print("qcurrent is in collision")

    # Tuck arm.
    res, qtuck, err = graph.generateTargetConfig('end_arm', qcurrent, qcurrent)
    if not res: print("failed to tuck arm")
    if not robot.configIsValid(qtuck): print("qtuck invalid")
    tuckpath = armPlanner.computePath(qcurrent, qtuck, resetRoadmap=True)
    #tuckpath = armPlanner.timeParameterization(tuckpath)
    setRobotJointBounds("planning")
    # Move mobile base.
    qhome = clusters[i_cluster][0][1][:4] + qtuck[4:]
    res, qhome, err = graph.generateTargetConfig('move_base', qhome, qhome)
    if not res: print("failed to move base")
    basepath = basePlanner.computePath(qtuck, qhome, resetRoadmap=not basePlannerUsePrecomputedRoadmap)

    # Move the head.
    # Compute config where the robot looks at the part
    constraints = wd(armPlanner.cconstraints.getConfigProjector().copy())
    constraints.add(look_at_part, 0)
    constraints.setRightHandSideFromConfig(qhome)
    res, qend = constraints.apply(qhome)
    if not res: print("failed to look at the part. It may not be visible.")
    headpath = armPlanner.computePath(qhome, qend, resetRoadmap=True)
    #headpath = armPlanner.timeParameterization(headpath)

    if outOfCollisionPath is not None:
        path = outOfCollisionPath.asVector()
        path.concatenate(tuckpath)
    else:
        path = tuckpath
    path.concatenate(basepath)
    path.concatenate(headpath)
    return path

def compute_path_for_cluster(i_cluster, qcurrent = None):
    if qcurrent is None:
        import estimation 
        qcurrent = estimation.get_current_robot_and_cylinder_config(robot, graph, q0[:])

    cluster = clusters[i_cluster]
    print(i_cluster)
    print(cluster)
    # Recompute a configuration for each handle
    new_cluster = []
    setRobotJointBounds("grasp-generation")
    for hi, qphi, qhi in cluster:
        ok, qphi2, qhi2 = generate_valid_config_for_handle(hi, qcurrent,
                qguesses = [qphi,qcurrent,] + [ qpg for _, qpg, qg in new_cluster ],
                NrandomConfig=15)
        if ok:
            new_cluster.append( (hi, qphi2, qhi2) )
        else:
            print("Could not reach handle", hi)

    setRobotJointBounds("planning")
    if len(new_cluster) == 0: return None, None

    paths = clusters_comp.solveTSP(armPlanner, new_cluster, qhome=qcurrent)
    path = concatenate_paths(paths)
    #optpath = armPlanner.timeParameterization(path)
    return new_cluster, path
# 2}}}

# vim: foldmethod=marker foldlevel=1
