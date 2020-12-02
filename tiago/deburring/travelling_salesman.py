from __future__ import print_function
from CORBA import Any, TC_long
from hpp.corbaserver.manipulation import Robot, loadServerPlugin, createContext, newProblem, ProblemSolver, ConstraintGraph, Rule, Constraints, CorbaClient
from hpp.gepetto.manipulation import ViewerFactory
import sys, argparse, numpy as np, time, tqdm

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

#Robot.urdfFilename = "package://tiago_data/robots/tiago_steel_without_wheels.urdf"
#Robot.srdfFilename = "package://tiago_data/srdf/tiago.srdf"
from hpp.rostools import process_xacro, retrieve_resource
Robot.urdfString = process_xacro("package://tiago_description/robots/tiago.urdf.xacro", "robot:=steel", "end_effector:=schunk-wsg")
with open(retrieve_resource("package://tiago_data/srdf/tiago.srdf"), "r") as f:
    Robot.srdfString = f.read()

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
def wd(o):
    from hpp.corbaserver import wrap_delete
    return wrap_delete(o, client.basic._tools)
client.manipulation.problem.selectProblem (args.context)

robot = Robot("robot", "tiago", rootJointType="planar", client=client)
robot.insertRobotSRDFModel("tiago", "tiago_data", "schunk", "_gripper")
robot.setJointBounds('tiago/root_joint', [-2, 2, -2, 2])
ps = ProblemSolver(robot)
vf = ViewerFactory(ps)

ps.selectPathValidation("Graph-Dichotomy", 0)
ps.selectPathProjector("Progressive", 0.2)
ps.addPathOptimizer("EnforceTransitionSemantic")
ps.addPathOptimizer("SimpleTimeParameterization")
if isSimulation:
    ps.setMaxIterProjection (1)

ps.setParameter("SimpleTimeParameterization/safety", 0.25)
ps.setParameter("SimpleTimeParameterization/order", 2)
ps.setParameter("SimpleTimeParameterization/maxAcceleration", 1.0)
ps.setParameter("ManipulationPlanner/extendStep", 0.7)

vf.loadObjectModel (Cylinder, "part")
robot.setJointBounds('part/root_joint', [-2, 2, -2, 2, -2, 2])

shrinkJointRange(robot, 0.95)

q0 = robot.getCurrentConfig()
q0[:4] = [0, -0.9, 0, 1]
q0[robot.rankInConfiguration['tiago/torso_lift_joint']] = 0.15
q0[robot.rankInConfiguration['tiago/arm_1_joint']] = 0.10
q0[robot.rankInConfiguration['tiago/arm_2_joint']] = -1.47
q0[robot.rankInConfiguration['tiago/arm_3_joint']] = -0.16
q0[robot.rankInConfiguration['tiago/arm_4_joint']] = 1.87
q0[robot.rankInConfiguration['tiago/arm_5_joint']] = -1.57
q0[robot.rankInConfiguration['tiago/arm_6_joint']] = 0.01
q0[robot.rankInConfiguration['tiago/arm_7_joint']] = 0.00

q0[robot.rankInConfiguration['tiago/gripper_finger_joint']] = \
        q0[robot.rankInConfiguration['tiago/gripper_right_finger_joint']] = 0.002

q0[robot.rankInConfiguration['part/root_joint']:] = [0,0,0.5,0,0,0,1]

def lockJoint(jname, q, cname=None):
    if cname is None:
        cname = jname
    s = robot.rankInConfiguration[jname]
    e = s+robot.getJointConfigSize(jname)
    ps.createLockedJoint(cname, jname, q[s:e])
    ps.setConstantRightHandSide(cname, True)
    return cname

ljs = list()
ps.createLockedJoint("tiago_base", "tiago/root_joint", [0,0,1,0])

ljs.append(lockJoint("part/root_joint", q0, "lock_part"))

for n in robot.jointNames:
    if n.startswith('tiago/gripper_'):
        ljs.append(lockJoint(n, q0))

lock_arm = [ lockJoint(n, q0) for n in robot.jointNames
        if n.startswith("tiago/arm") or n.startswith('tiago/torso')]

ps.createPositionConstraint("gaze", "tiago/xtion_rgb_optical_frame", "tiago/gripper",
        (0,0,0), (0,0,0), (True,True,False))

from hpp.corbaserver.manipulation import ConstraintGraphFactory
graph = ConstraintGraph(robot, 'graph')
factory = ConstraintGraphFactory(graph)
factory.setGrippers([ "tiago/gripper",])

handles = ps.getAvailable('handle')
factory.setObjects([ "part", ], [ handles ], [ [ ], ])

factory.setRules([ Rule([ "tiago/gripper", ], [ ".*", ], True), ])
factory.generate()

graph.addConstraints(graph=True, constraints=Constraints(numConstraints=ljs))
for n in graph.nodes.keys():
    if n == "free": continue
    graph.addConstraints(node=n, constraints=Constraints(numConstraints=["gaze"]))
for e in graph.edges.keys():
    graph.addConstraints(edge=e, constraints=Constraints(numConstraints=["tiago_base"]))

graph.createNode('home', priority=1000)
graph.createEdge('home', 'home', 'move_base')
graph.createEdge('home', 'free', 'start_arm', isInNode="home")
graph.createEdge('free', 'home', 'end_arm', isInNode="free")

graph.addConstraints(node="home", constraints=Constraints(numConstraints=lock_arm))
graph.addConstraints(edge="end_arm", constraints=Constraints(numConstraints=["tiago_base"]))

graph.initialize()

graphValidation = ps.client.manipulation.problem.createGraphValidation()
graphValidation.validate(ps.hppcorba.problem.getProblem().getConstraintGraph())
if graphValidation.hasErrors():
    print(graphValidation.str())
    print("Graph has infeasibilities")
elif graphValidation.hasWarnings():
    print(graphValidation.str())
    print("Graph has only warnings")
else:
    print("Graph *seems* valid.")

def find_cluster(hi, handles):
    #ni = "tiago/gripper grasps " + hi
    ni = "tiago/gripper > " + hi + " | f_pregrasp"
    qrand = q0
    best_cluster = []
    for _ in range(4):
        res, q1, err = graph.applyNodeConstraints(ni, qrand)
        if res and robot.isConfigValid(q1)[0]:
            t_base = np.array(robot.getJointPosition('tiago/torso_fixed_joint')[:3])
            cur_cluster = [ (hi, q1) ]
            for hj in handles:
                if hj == hi: continue
                t_handle = np.array(robot.getHandlePositionInJoint(hi)[1][:3])
                if False and np.linalg.norm(t_handle-t_base) > 0.6:
                    #print("prune")
                    continue
                ei = "tiago/gripper > " + hj
                qrand = q1
                ok = False
                for k in range(10):
                    res, q2, err = graph.generateTargetConfig (ei+" | f_01", q1, qrand)
                    if res:
                        ok, msg = robot.isConfigValid(q2)
                        if ok: break
                    qrand = robot.shootRandomConfig()
                if ok:
                    cur_cluster.append((hj, q2))
            if len(cur_cluster) > len(best_cluster):
                best_cluster = cur_cluster
        qrand = robot.shootRandomConfig()
    return best_cluster

def find_clusters(handles):
    start = time.time()
    clusters = []
    #remaining_handles = handles.copy()
    remaining_handles = handles[:]
    while remaining_handles:
        cluster = find_cluster(remaining_handles[0], remaining_handles)
        for hi, qi in cluster:
            remaining_handles.remove(hi)
        clusters.append(cluster)
    print(len(clusters), "clusters found in", time.time()-start)
    return clusters

class InStatePlanner:
    # Default path planner
    plannerType = "BiRRT*"
    optimizerTypes = []
    maxIterPathPlanning = None
    parameters = {'kPRM*/numberOfNodes': Any(TC_long,2000)}

    def __init__(self):

        manipulationProblem = wd(ps.hppcorba.problem.getProblem())
        self.crobot = manipulationProblem.robot()
        # create a new problem with the robot
        self.cproblem = wd(ps.hppcorba.problem.createProblem(self.crobot))
        # Set parameters
        for k, v in self.parameters.items():
            self.cproblem.setParameter(k, v)
        # get reference to constraint graph
        self.cgraph = manipulationProblem.getConstraintGraph()
        # Add obstacles to new problem
        for obs in ps.getObstacleNames(True,False):
            self.cproblem.addObstacle(wd(ps.hppcorba.problem.getObstacle(obs)))

    def setEdge(self, edge):
        # Get constraint of edge
        edgeLoopFree = wd(self.cgraph.get(graph.edges[edge]))
        self.cconstraints = wd(edgeLoopFree.pathConstraint())
        self.cproblem.setConstraints(self.cconstraints)
        self.cproblem.setSteeringMethod(wd(edgeLoopFree.getSteeringMethod()))
        self.cproblem.filterCollisionPairs()

    def setReedsAndSheppSteeringMethod(self):
        sm = wd(ps.hppcorba.problem.createSteeringMethod("ReedsShepp", self.cproblem))
        self.cproblem.setSteeringMethod(sm)
        dist = ps.hppcorba.problem.createDistance("ReedsShepp", self.cproblem)
        self.cproblem.setDistance(dist)

    def computePath(self, qinit, qgoal):
        wd(self.cconstraints.getConfigProjector()).setRightHandSideFromConfig(qinit)
        self.cproblem.setInitConfig(qinit)
        self.cproblem.resetGoalConfigs()
        if not qgoal is None:
            res, qgoal2 = self.cconstraints.apply(qgoal)
            self.cproblem.addGoalConfig(qgoal2)

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
        path = wd(self.planner.solve())
        if not path: return None
        for optType in self.optimizerTypes:
            from hpp_idl.hpp import Error
            optimizer = wd(ps.hppcorba.problem.createPathOptimizer(optType, self.cproblem))
            try:
                path = wd(optimizer.optimize(path))
            except Error as e:
                print(e)
        return path

    def solveTSP(self, configs):
        # Compute matrix of paths
        l = len(configs)
        paths = [ [ None, ] * l for _ in range(l) ]
        distances = np.zeros((l,l))
        for i in range(l):
            for j in range(i+1,l):
                if False:
                    path = paths[i][j] = self.computePath(configs[i],configs[j])
                    distances[j,i] = distances[i,j] = path.length()
                else:
                    try:
                        path = paths[i][j] = self.computePath(configs[i],configs[j])
                        distances[j,i] = distances[i,j] = path.length()
                    except:
                        print("Failed to connect", i, "to", j)
                        paths[i][j] = None
                        distances[j,i] = distances[i,j] = 1e8
        if sys.version_info[0] == 2:
            from tsp import dynamic_programming as solve_tsp
        elif sys.version_info[0] == 3:
            if l <= 10:
                from python_tsp.exact import solve_tsp_dynamic_programming as solve_tsp
            else:
                from python_tsp import simulated_annealing as solve_tsp
        permutation, distance = solve_tsp(distances)
        # rotate permutation so that 0 is the first index.
        k = permutation.index(0)
        permutation = permutation[k:] + permutation[:k]
        #print (distance)
        solution = []
        permutation.append(0)
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
    p = paths[0].asVector()
    for q in paths[1:]:
        p.appendPath(q)
    return p

armPlanner = InStatePlanner ()
armPlanner.setEdge("Loop | f")

basePlanner = InStatePlanner ()
basePlanner.plannerType = "kPRM*"
basePlanner.cproblem.setParameter('kPRM*/numberOfNodes', Any(TC_long,2000))
basePlanner.optimizerTypes.append("RandomShortcut")
basePlanner.setEdge("move_base")
basePlanner.setReedsAndSheppSteeringMethod()
#basePlanner.computePath(q0,None)
