#/usr/bin/env python
from math import sqrt
from hpp.corbaserver.manipulation.robot import Robot
from hpp.corbaserver.manipulation import ProblemSolver, ConstraintGraph, Rule
from hpp.gepetto.manipulation import ViewerFactory
from datetime import datetime
a = sqrt (2)/2

Robot.packageName = "talos_data"
Robot.urdfName = "talos"
Robot.urdfSuffix = '_full_v2'
Robot.srdfSuffix= ''

class Box (object):
  def __init__ (self, name, vf) :
    self.name = name
    self.handles = [ name + "/" + h for h in self.__class__.handles ]
    self.contacts = [ name + "/" + h for h in self.__class__.contacts ]
    vf.loadObjectModel (self.__class__, name)

  rootJointType = 'freeflyer'
  packageName = 'gerard_bauzil'
  urdfName = 'plank_of_wood1'
  urdfSuffix = ""
  srdfSuffix = ""
  handles = ["handle1", "handle2", "handle3", "handle4"]
  contacts = [ "front_surface", "rear_surface", ]

class Table (object):
  def __init__ (self, name, vf) :
    self.name = name
    self.handles = [ name + "/" + h for h in self.__class__.handles ]
    self.contacts = [ name + "/" + h for h in self.__class__.contacts ]
    vf.loadObjectModel (self.__class__, name)

  rootJointType = 'freeflyer'
  packageName = 'gerard_bauzil'
  urdfName = 'table_140_70_73'
  urdfSuffix = ""
  srdfSuffix = ""
  pose = "pose"
  handles = []
  contacts = [ "top", ]

half_sitting = [
        # -0.74,0,1.0192720229567027,0,0,0,1, # root_joint
        0.6,-0.65,1.0192720229567027,0,0,sqrt(2)/2,sqrt(2)/2, # root_joint
        0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, # leg_left
        0.0, 0.0, -0.411354, 0.859395, -0.448041, -0.001708, # leg_right
        0, 0.006761, # torso
        0.25847, 0.173046, -0.0002, -0.525366, 0, 0, 0.1, # arm_left
        0, 0, 0, 0, 0, 0, 0, # gripper_left
        -0.25847, -0.173046, 0.0002, -0.525366, 0, 0, 0.1, # arm_right
        0, 0, 0, 0, 0, 0, 0, # gripper_right
        0, 0, # head

        -0.04,0,1.095+0.071,0,0,1,0, # box
        0, 0, 0, 0, 0, 0, 1, # table
        ]

# Load an object of type Type with name name
def loadObject (vf, Type, name) :
    vf.loadObjectModel (Type, name)
    obj = Type (name)
    obj.handles = [ name + "/" + h for h in Type.handles ]
    obj.contacts = [ name + "/" + h for h in Type.contacts ]
    return obj

def makeRobotProblemAndViewerFactory (clients):
    objects = list ()
    robot = Robot ('talos', 'talos', rootJointType = "freeflyer",
                   client = clients)
    robot. leftAnkle = "talos/leg_left_6_joint"
    robot.rightAnkle = "talos/leg_right_6_joint"

    robot.setJointBounds ("talos/root_joint", [-10, 10, -10, 10, 0, 2])

    ps = ProblemSolver (robot)
    ps.setErrorThreshold (1e-3)
    ps.setMaxIterProjection (40)

    ps.addPathOptimizer("SimpleTimeParameterization")

    vf = ViewerFactory (ps)
    objects.append (Box (name = 'box', vf = vf))
    robot.setJointBounds ("box/root_joint", [-10, 10, -10, 10, 0, 2])

    # Loaded as an object to get the visual tags at the right position.
    # vf.loadEnvironmentModel (Table, 'table')
    table = Table (name = 'table', vf = vf)
    robot.setJointBounds ("table/root_joint", [-10, 10, -10, 10, -2, 2])

    return robot, ps, vf, table, objects

def makeGraph (robot, table, objects):
    from hpp.corbaserver.manipulation.constraint_graph_factory import \
      ConstraintGraphFactory
    graph = ConstraintGraph(robot, 'graph')
    factory = ConstraintGraphFactory (graph)
    factory.setGrippers ([ "talos/left_gripper", "talos/right_gripper"])
    factory.setObjects ([ obj.name for obj in objects],
                        [ obj.handles for obj in objects ],
                        [ obj.contacts for obj in objects])
    factory.environmentContacts (table.contacts)
    factory.constraints.strict = True
    factory.generate ()
    return graph

def shootConfig (robot, q, i):
  """
  Shoot a random config if i > 0, return input configuration otherwise
  """
  if i==0: return q
  return robot.shootRandomConfig ()

def createConnection (ps, graph, e, q, maxIter):
  """
  Try to build a path along a transition from a given configuration
  """
  for i in range (maxIter):
    q_rand = shootConfig (ps.robot, q, i)
    res, q1, err = graph.generateTargetConfig (e, q, q_rand)
    if not res: continue
    res, p, msg = ps.directPath (q, q1, True)
    if not res: continue
    ps.addConfigToRoadmap (q1)
    ps.addEdgeToRoadmap (q, q1, p, True)
    print ("Success (i={0})".format (i))
    return p, q1
  print ("Failed  (maxIter={0})".format (maxIter))
  return (None, None)

class Solver (object):
  """
  Solver that tries direct connections before calling RRT.
  """
  def __init__ (self, ps, graph, q_init, q_goal,
                 e_l1, e_r1, e_l2, e_r2, e_l3, e_r3, e_l4, e_r4,
                 e_l1_r2, e_l1_r4, e_r1_l2, e_r1_l4, e_l2_r1, e_l2_r3,
                 e_r2_l1, e_r2_l3, e_r3_l4, e_r3_l2, e_l3_r4, e_l3_r2,
                 e_l4_r1, e_l4_r3, e_r4_l1, e_r4_l3) :
    self.ps = ps; self.graph = graph
    self.e_l1 = e_l1; self.e_r1 = e_r1;
    self.e_l2 = e_l2; self.e_r2 = e_r2;
    self.e_l3 = e_l3; self.e_r3 = e_r3;
    self.e_l4 = e_l4; self.e_r4 = e_r4;
    self.e_l1_r2 = e_l1_r2; self.e_l1_r4 = e_l1_r4;
    self.e_r1_l2 = e_r1_l2; self.e_r1_l4 = e_r1_l4;
    self.e_l2_r1 = e_l2_r1; self.e_l2_r3 = e_l2_r3;
    self.e_r2_l1 = e_r2_l1; self.e_r2_l3 = e_r2_l3;
    self.e_r3_l4 = e_r3_l4; self.e_r3_l2 = e_r3_l2;
    self.e_l3_r4 = e_l3_r4; self.e_l3_r2 = e_l3_r2;
    self.e_l4_r1 = e_l4_r1; self.e_l4_r3 = e_l4_r3;
    self.e_r4_l1 = e_r4_l1; self.e_r4_l3 = e_r4_l3;
    self.q_init = q_init; self.q_goal = q_goal

  def addWaypoints (self, config):
    e = 'Loop | f'
    robot = self.ps.robot
    rank1 = robot.rankInConfiguration ['talos/arm_left_4_joint']
    rank2 = robot.rankInConfiguration ['talos/arm_right_4_joint']
    q = config [::]
    # move left elbow
    q [rank1] = -1.7
    q [rank2] = -1.7
    # Project q on state 'free'
    res, wp, err = self.graph.generateTargetConfig (e, config, q)
    if res:
      # test collision for wp
      res, msg = robot.isConfigValid (wp)
      if res:
        # call steering method
        res, p, msg = self.ps.directPath (config, wp, True)
        if res:
          # add node and edge
          self.ps.addConfigToRoadmap (wp)
          self.ps.addEdgeToRoadmap (config, wp, p, True)
          # store wp
          return wp
    return config

  def tryDirectPaths (self, possibleConnections) :
    for q1, q2 in possibleConnections:
      if q1 and q2:
        res, p, msg = self.ps.directPath (q1, q2, True)
        if res:
          print ("direct connection succeeded")
          self.ps.addEdgeToRoadmap (q1, q2, p, True)
        else:
          print ("failed direct connection: " + msg)
          print ("q1= " + str(q1))
          print ("q2= " + str(q2))

  def solve (self):
    import numpy as np
    assert np.linalg.norm(np.array(self.q_init[-7:]) - np.array(self.q_goal[-7:])) < 1e-7
    start = datetime.now ()
    q_l1_r2 = None; q_r2_l1 = None;q_l1_r4 = None; q_r4_l1 = None
    q_r1_l2 = None; q_l2_r1 = None;q_r1_l4 = None; q_l4_r1 = None
    q_l2_r3 = None; q_r3_l2 = None;q_r2_l3 = None; q_l3_r2 = None
    q_l3_r4 = None; q_r4_l3 = None;q_r3_l4 = None; q_l4_r3 = None
    self.ps.addConfigToRoadmap (self.q_init)
    self.ps.addConfigToRoadmap (self.q_goal)

    print ("Generating init waypoint.")
    self.wp_init = self.addWaypoints (self.q_init)
    print ("Generating goal waypoint.")
    self.wp_goal = self.addWaypoints (self.q_goal)

    ## Connections from init to grasp
    print "Edge e_l1   : ",
    p, q_l1 = createConnection \
               (self.ps, self.graph, self.e_l1, self.wp_init, 20)
    print "Edge e_r1   : ",
    p, q_r1 = createConnection \
               (self.ps, self.graph, self.e_r1, self.wp_init, 20)
    print "Edge e_l3   : ",
    p, q_l3 = createConnection \
               (self.ps, self.graph, self.e_l3, self.wp_init, 20)
    print "Edge e_r3   : ",
    p, q_r3 = createConnection \
               (self.ps, self.graph, self.e_r3, self.wp_init, 20)
    ## Connections from goal to grasp
    print "Edge e_l4   : ",
    p, q_l4 = createConnection \
               (self.ps, self.graph, self.e_l4, self.wp_goal, 20)
    print "Edge e_r4   : ",
    p, q_r4 = createConnection \
               (self.ps, self.graph, self.e_r4, self.wp_goal, 20)
    print "Edge e_l2   : ",
    p, q_l2 = createConnection \
               (self.ps, self.graph, self.e_l2, self.wp_goal, 20)
    print "Edge e_r2   : ",
    p, q_r2 = createConnection \
               (self.ps, self.graph, self.e_r2, self.wp_goal, 20)
    ## Connections from one grasp to two grasps
    if q_l1:
      print "Edge e_l1_r2: ",
      p, q_l1_r2 = createConnection \
                   (self.ps, self.graph, self.e_l1_r2, q_l1, 20)
      print "Edge e_l1_r4: ",
      p, q_l1_r4 = createConnection \
                   (self.ps, self.graph, self.e_l1_r4, q_l1, 20)
    if q_r1:
      print "Edge e_r1_l2: ",
      p, q_r1_l2 = createConnection \
                   (self.ps, self.graph, self.e_r1_l2, q_r1, 20)
      print "Edge e_r1_l4: ",
      p, q_r1_l4 = createConnection \
                   (self.ps, self.graph, self.e_r1_l4, q_r1, 20)
    if q_l2:
      print "Edge e_l2_r1: ",
      p, q_l2_r1 = createConnection \
                   (self.ps, self.graph, self.e_l2_r1, q_l2, 20)
      print "Edge e_l2_r3: ",
      p, q_l2_r3 = createConnection \
                   (self.ps, self.graph, self.e_l2_r3, q_l2, 20)
    if q_r2:
      print "Edge e_r2_l1: ",
      p, q_r2_l1 = createConnection \
                   (self.ps, self.graph, self.e_r2_l1, q_r2, 20)
      print "Edge e_r2_l3: ",
      p, q_r2_l3 = createConnection \
                   (self.ps, self.graph, self.e_r2_l3, q_r2, 20)
    if q_l3:
      print "Edge e_l3_r4: ",
      p, q_l3_r4 = createConnection \
                   (self.ps, self.graph, self.e_l3_r4, q_l3, 20)
      print "Edge e_l3_r2: ",
      p, q_l3_r2 = createConnection \
                   (self.ps, self.graph, self.e_l3_r2, q_l3, 20)
    if q_r3:
      print "Edge e_r3_l4: ",
      p, q_r3_l4 = createConnection \
                   (self.ps, self.graph, self.e_r3_l4, q_r3, 20)
      print "Edge e_r3_l2: ",
      p, q_r3_l2 = createConnection \
                   (self.ps, self.graph, self.e_r3_l2, q_r3, 20)
    if q_l4:
      print "Edge e_l4_r1: ",
      p, q_l4_r1 = createConnection \
                   (self.ps, self.graph, self.e_l4_r1, q_l4, 20)
      print "Edge e_l4_r3: ",
      p, q_l4_r3 = createConnection \
                   (self.ps, self.graph, self.e_l4_r3, q_l4, 20)
    if q_r4:
      print "Edge e_r4_l1: ",
      p, q_r4_l1 = createConnection \
                   (self.ps, self.graph, self.e_r4_l1, q_r4, 20)
      print "Edge e_r4_l3: ",
      p, q_r4_l3 = createConnection \
                   (self.ps, self.graph, self.e_r4_l3, q_r4, 20)

    possibleConnections = [(q_l1_r2, q_r2_l1),(q_l1_r4, q_r4_l1),
                           (q_r1_l2, q_l2_r1),(q_r1_l4, q_l4_r1),
                           (q_l2_r3, q_r3_l2),(q_r2_l3, q_l3_r2),
                           (q_l3_r4, q_r4_l3),(q_r3_l4, q_l4_r3),]
    self.tryDirectPaths (possibleConnections)

    self.ps.setInitialConfig (self.q_init)
    self.ps.resetGoalConfigs ()
    self.ps.addGoalConfig (self.q_goal)
    print ("Running Manipulation RRT")
    self.ps.solve ()
    end = datetime.now ()
    print ("Resolution time : {0}". format (end - start))

  def makeBoxVisibleFrom (self, q_estimated, initObjectPose, initTablePose):
    q = q_estimated[:]
    rankO = self.ps.robot.rankInConfiguration ['box/root_joint']
    rankT = self.ps.robot.rankInConfiguration ['table/root_joint']
    if initObjectPose:
      q[rankO:rankO+7] = self.q_init[rankO:rankO+7]
    if initTablePose:
      q[rankT:rankT+7] = self.q_init[rankT:rankT+7]

    res, q_proj, err = self.graph.generateTargetConfig ("loop_ss", q, q)
    assert res, "Failed: generateTargetConfig loop_ss"
    res, qres, err = self.graph.generateTargetConfig ("starting_motion", q_proj, q_proj)
    assert res, "Failed: generateTargetConfig starting_motion"
    res, pid, msg = self.ps.directPath (q_proj, qres, True)
    self.ps.addConfigToRoadmap (q_proj)
    self.ps.addConfigToRoadmap (qres)
    self.ps.addEdgeToRoadmap (q_proj, qres, pid, True)
    if res:
      self.ps.optimizePath(pid)
      print "Initial path", pid+1
      return qres, pid+1
    else:
      print "Failed: directPath", msg
      return qres, None

  # \param qEstimated box should be visible.
  def generateGoalFrom (self, qEstimated, qDesiredRobot):
    qgoal = qEstimated[:]
    rankO = self.ps.robot.rankInConfiguration ['box/root_joint']
    rankT = self.ps.robot.rankInConfiguration ['table/root_joint']
    # Rotate the box.
    from hpp import Quaternion
    qT = Quaternion (qEstimated[rankO+3:rankO+7])
    qgoal[rankO+3:rankO+7] = (qT * Quaternion([0, 0, 1, 0])).toTuple()
    res, qgoal, err = self.graph.generateTargetConfig ("starting_motion", qgoal, qgoal)
    success = ("free" == self.graph.getNode (qgoal))
    if not res or not success:
      print "Could not generate goal"
    qgoalInStartingState = qDesiredRobot[:min(rankO,rankT)] + qgoal[min(rankO,rankT):]
    #TODO if a transition from free to starting_state is added
    # change the order of the arguments.
    #res, pid, msg = ps.directPath (qgoal, qgoalInStartingState, True)
    #self.tryDirectPaths (((qgoalInStartingState, qgoal),))
    return qgoal, qgoalInStartingState

  def acquireEstimation (self, topic="/agimus/estimation/semantic"):
    boxSizeZ = 0.203
    tableRealHeight = 0.735
    boxExpectedZ = tableRealHeight + boxSizeZ / 2

    import rospy, numpy as np
    from dynamic_graph_bridge_msgs.msg import Vector
    msg = rospy.wait_for_message (topic, Vector, timeout=2)
    qestimated = list(msg.data)
    # Fix estimation
    self.ps.robot.setCurrentConfig (qestimated)
    p_cam = np.array(self.ps.robot.getJointPosition ("talos/rgbd_rgb_optical_joint")[0:3])
    p_obj = np.array(self.ps.robot.getJointPosition ("box/root_joint")[0:3])
    p_cam_obj = p_cam - p_obj

    ric = self.ps.robot.rankInConfiguration
    qestimated[ric["talos/gripper_right_joint"]]=0
    qestimated[ric["talos/gripper_left_joint"]]=0
    curBoxZ = qestimated[ric["box/root_joint"]+2]
    z_corr = boxExpectedZ - curBoxZ

    x_corr = z_corr / p_cam_obj[2] * p_cam_obj[0]
    y_corr = z_corr / p_cam_obj[2] * p_cam_obj[1]
    print "Correction on X (NOT applied) axis:", x_corr
    print "Correction on Y (    applied) axis:", y_corr
    print "Correction on Z (    applied) axis:", z_corr
    #qestimated[ric["box/root_joint"]  +0] +=x_corr
    #qestimated[ric["table/root_joint"]+0] +=x_corr
    qestimated[ric["box/root_joint"]  +1] +=y_corr
    qestimated[ric["table/root_joint"]+1] +=y_corr
    qestimated[ric["box/root_joint"]  +2] +=z_corr
    qestimated[ric["table/root_joint"]+2] +=z_corr

    return qestimated

  def initRosNode (self):
    import rospy
    rospy.init_node("hpp_script", anonymous=True)

  def solveFromEstimatedConfiguration (self, half_sitting, q_estimated = None):
    if q_estimated == None:
      self.q_estimated = self.acquireEstimation ()
    else:
      self.q_estimated = q_estimated
    self.q_init, initial_path_id = self.makeBoxVisibleFrom (self.q_estimated, False, False)
    self.q_goal, self.q_goal_in_starting_state = self.generateGoalFrom (self.q_estimated, half_sitting)
    self.solve()
    print "Initial path: ", initial_path_id
    print "Path to achieve the box goal position: ", self.ps.numberPaths() - 1
    res, pid, msg = self.ps.directPath (self.q_goal, self.q_goal_in_starting_state, True)
    if res:
      self.ps.optimizePath (pid)
      print "Path to go back to half_sitting:", self.ps.numberPaths() - 1
    else:
      print "Cannot go back to half_sitting:", msg
