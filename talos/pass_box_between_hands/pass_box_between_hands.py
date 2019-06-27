# /usr/bin/env python
import numpy as np
from hpp import Transform
from hpp.corbaserver.manipulation import (
    ConstraintGraph,
    ProblemSolver,
    Rule,
    newProblem,
)
from hpp.corbaserver.manipulation.robot import Robot
from hpp.gepetto.manipulation import ViewerFactory


newProblem()

Robot.packageName = "talos_data"
Robot.urdfName = "talos"
Robot.urdfSuffix = "_full"
Robot.srdfSuffix = ""


class Box(object):
    rootJointType = "freeflyer"
    packageName = "agimus_demos"
    urdfName = "cup"
    urdfSuffix = ""
    srdfSuffix = ""
    handles = ["box/top", "box/bottom"]


robot = Robot("dev", "talos", rootJointType="freeflyer")
robot.leftAnkle = "talos/leg_left_6_joint"
robot.rightAnkle = "talos/leg_right_6_joint"

robot.setJointBounds("talos/root_joint", [-1, 1, -1, 1, 0.5, 1.5])

ps = ProblemSolver(robot)
ps.setRandomSeed(123)
ps.selectPathProjector("Progressive", 0.2)
ps.setErrorThreshold(1e-3)
ps.setMaxIterProjection(40)

ps.addPathOptimizer("SimpleTimeParameterization")

vf = ViewerFactory(ps)
Object = Box
vf.loadObjectModel(Object, "box")
robot.setJointBounds("box/root_joint", [-1, 1, -1, 1, 0, 2])

qq = [
    -0.7671778026566639,
    0.0073267002287253635,
    1.0035168727631776,
    -0.003341673452654457,
    -0.021566597515109524,
    0.0002183620894239602,
    0.9997618053357284,
    -0.00020053128587844821,
    -0.0021365695604276275,
    -0.4415951773681094,
    0.9659230706255528,
    -0.48119003672520416,
    0.007109157982067145,
    -0.00020095991543181877,
    -0.002126639473414498,
    -0.4382848597339842,
    0.9589221865248464,
    -0.4774994711722908,
    0.007099218648561522,
    0.0,
    0.025235347910697536,
    -0.06985947194357875,
    0.0,
    -0.12446173084176845,
    -1.5808415926365578,
    0.014333078135875619,
    -0.0806417043955706,
    -0.37124401660668394,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    -0.25955282922987977,
    -0.1618313202464181,
    0.002447883426630002,
    -0.5149037074691503,
    -0.00010703274362664899,
    0.0008742582163227642,
    0.10168585913285667,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.785398163397,
    0.32250041955468695,
    -0.2569883469655496,
    0.18577095561452217,
    1.164388709412583,
    0.0694401264431558,
    0.5475575114527793,
    -0.11842286843715424,
    0.8254301089264399,
]

half_sitting = [
    -0.74,
    0,
    1.0192720229567027,
    0,
    0,
    0,
    1,  # root_joint
    0.0,
    0.0,
    -0.411354,
    0.859395,
    -0.448041,
    -0.001708,  # leg_left
    0.0,
    0.0,
    -0.411354,
    0.859395,
    -0.448041,
    -0.001708,  # leg_right
    0,
    0.006761,  # torso
    0.25847,
    0.173046,
    -0.0002,
    -0.525366,
    0,
    0,
    0.1,  # arm_left
    0,
    0,
    0,
    0,
    0,
    0,
    0,  # gripper_left
    -0.25847,
    -0.173046,
    0.0002,
    -0.525366,
    0,
    0,
    0.1,  # arm_right
    0,
    0,
    0,
    0,
    0,
    0,
    0,  # gripper_right
    0,
    0,  # head
    0,
    0,
    0,
    0,
    0,
    0,
    1,  # box
]
q_init = robot.getCurrentConfig()

ps.addPartialCom("talos", ["talos/root_joint"])
ps.addPartialCom("talos_box", ["talos/root_joint", "box/root_joint"])

ps.createStaticStabilityConstraints(
    "balance", half_sitting, "talos", ProblemSolver.FIXED_ON_THE_GROUND
)
foot_placement = ["balance/pose-left-foot", "balance/pose-right-foot"]
foot_placement_complement = []

robot.setCurrentConfig(half_sitting)
com_wf = np.array(ps.getPartialCom("talos"))
tf_la = Transform(robot.getJointPosition(robot.leftAnkle))
com_la = tf_la.inverse().transform(com_wf)

ps.createRelativeComConstraint(
    "com_talos_box", "talos_box", robot.leftAnkle, com_la.tolist(), (True, True, True)
)
ps.createRelativeComConstraint(
    "com_talos", "talos", robot.leftAnkle, com_la.tolist(), (True, True, True)
)

ps.createPositionConstraint(
    "gaze",
    "talos/rgbd_optical_joint",
    "box/root_joint",
    (0, 0, 0),
    (0, 0, 0),
    (True, True, False),
)

left_gripper_lock = []
right_gripper_lock = []
other_lock = ["talos/torso_1_joint"]
for n in robot.jointNames:
    s = robot.getJointConfigSize(n)
    r = robot.rankInConfiguration[n]
    if n.startswith("talos/gripper_right"):
        ps.createLockedJoint(n, n, half_sitting[r : r + s])
        right_gripper_lock.append(n)
    elif n.startswith("talos/gripper_left"):
        ps.createLockedJoint(n, n, half_sitting[r : r + s])
        left_gripper_lock.append(n)
    elif n in other_lock:
        ps.createLockedJoint(n, n, half_sitting[r : r + s])

graph = ConstraintGraph.buildGenericGraph(
    robot,
    "graph",
    ["talos/left_gripper", "talos/right_gripper"],
    ["box"],
    [Object.handles],
    [[]],
    [],
    [
        Rule(["talos/left_gripper"], [Object.handles[0]], True),
        Rule(["talos/right_gripper"], [Object.handles[1]], True),
    ],
)

graph.setConstraints(
    graph=True,
    lockDof=left_gripper_lock + right_gripper_lock + other_lock,
    numConstraints=["com_talos_box", "gaze"] + foot_placement,
)
graph.initialize()

res, q_init, err = graph.applyNodeConstraints(
    "talos/left_gripper grasps box/top", half_sitting
)
res, q_goal, err = graph.applyNodeConstraints(
    "talos/right_gripper grasps box/bottom", half_sitting
)
print(ps.directPath(q_init, q_init, True))
ps.setInitialConfig(q_init)
ps.addGoalConfig(q_goal)
ps.setParameter("SimpleTimeParameterization/safety", 0.5)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 2.0)

ps.solve()


def createInitializationGraph(name):
    graph = ConstraintGraph.buildGenericGraph(
        robot,
        name,
        ["talos/left_gripper", "talos/right_gripper"],
        ["box"],
        [Object.handles],
        [[]],
        [],
        [
            Rule(["talos/left_gripper"], [Object.handles[0]], True),
            Rule(["talos/right_gripper"], [Object.handles[1]], True),
        ],
    )
    return graph


def setGaussianShooter(q, dev):
    robot.setCurrentConfig(q)
    selected = ps.getSelected("configurationshooter")[0]
    ps.setParameter("ConfigurationShooter/Gaussian/standardDeviation", dev)
    ps.client.basic.problem.selectConfigurationShooter("Gaussian")
    return selected


def restoreProblem():
    # Restore the state of the problem
    ps.client.manipulation.graph.selectGraph(graph.name)
    ps.client.basic.problem.selectConfigurationShooter("Uniform")


def applySemantic(state, qsensor, dev):
    """
    Given an estimated state of the world, generate a configuration that
    make 'sense':
    - no collisions (between objects, robots and world)
    - the objects are either grasped or in stable placement.
    """

    setGaussianShooter(qsensor, dev)

    # Apply semantic information
    ps.client.manipulation.graph.selectGraph(initializationGraph.name)
    qsemantic = qsensor[:]
    while True:
        valid, qsemantic, err = initializationGraph.applyNodeConstraints(
            state, qsemantic
        )
        if valid:
            valid, msg = robot.isConfigValid(qsemantic)
            if valid:
                break
        qsemantic = robot.shootRandomConfig()

    restoreProblem()
    return qsemantic


def goToGraph(state, q):
    dev = 0.005
    qsemantic = applyObjectSemantic(state, q, dev)

    setGaussianShooter(qsemantic, dev)

    qgoal = qsemantic[:]
    # Try to generate a configuration from q
    while True:
        valid, qgoal, err = graph.applyNodeConstraints(state, qgoal)
        if valid:
            valid, msg = robot.isConfigValid(qgoal)
            if valid:
                break
        qgoal = robot.shootRandomConfig()

    # graphs = ps.getAvailable("constraintgraph")
    # if "initialization" in graphs:
    # newGraph = ConstraintGraph (robot, "initialization")
    # newGraph.createNode ("free")
    # newGraph.createEdge ("free", "free", "transition")
    # else:
    # ps.client.manipulation.graph.selectGraph("initialization")

    ps.client.manipulation.graph.selectGraph(initializationGraph.name)
    ps.resetGoalConfigs()
    ps.setInitialConfig(qsemantic)
    ps.addGoalConfig(qgoal)
    ps.solve()
    pid = ps.numberPaths() - 1

    # Restore the state of the problem
    restoreProblem()

    return pid


def testGoToGraph(state, q, dev):
    selected = setGaussianShooter(q, dev)
    qsensor = robot.shootRandomConfig()

    ps.client.basic.problem.selectConfigurationShooter(selected)
    pid = goToGraph(state, qsensor)

    return qsensor, pid


initializationGraph = createInitializationGraph("initialization")
ps.client.manipulation.graph.selectGraph(graph.name)

try:
    v = vf.createViewer()
except Exception as e:
    print("Could not connect to the viewer:", str(e))
