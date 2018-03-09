import sys
if not hasattr(sys, "argv"):
    sys.argv = []

class Client:
    from hpp.corbaserver import Client as BasicClient
    from hpp.corbaserver.manipulation import Client as ManipClient
    def __init__(self):
        self.basic = Client.BasicClient()
        self.manipulation = Client.ManipClient()

def hpTasks(sotrobot):
    from sot_hpp.tools import COM, Foot, Manifold
    com = COM ("talos", sotrobot)
    lf = Foot ("talos/leg_left_6_joint", sotrobot) # Feet placement make the robot fall down.
    rf = Foot ("talos/leg_right_6_joint", sotrobot)
    #lf = Foot3D ("talos/leg_left_6_joint", sotrobot) # Feet placement make the robot fall down.
    #rf = Foot3D ("talos/leg_right_6_joint", sotrobot)
    return com + lf + rf
    #return lf + rf + com
    # return com + lf
    # return com
    # return lf + rf
    # return Manifold()

def makeSupervisor (robot, edges):
    from sot_hpp import Supervisor

    hppclient = Client()

    grippers = [ "talos/left_gripper", "talos/right_gripper" ]
    objects = [ "box" ]
    handlesPerObjects = [ [ "box/top", "box/bottom" ], ]

    supervisor = Supervisor (robot)
    supervisor.initForGrasps(hppclient, grippers, objects, handlesPerObjects, hpTasks = hpTasks(robot))
    transitions = parseTransitionNames(edges.values(), edges.keys(), supervisor)
    supervisor.makeGrasps(transitions)
    supervisor.makeInitialSot ()
    return supervisor

def parseTransitionNames (ids, names, supervisor):
    import re
    # regexEdge = re.compile("^(?:(?P<robot>\w+)/(?P<gripper>\w+) (?<direction>[<>]) (?P<object>\w+)/(?P<handle>\w+)|Loop) \| (?P<manifold>f|[0-9-:]+)(?P<step>_[0-9]+)?(?P<levelset>_ls)?$")
    regexEdge = re.compile("^(?:(?P<gripper>[\w/]+) (?P<direction>[<>]) (?P<handle>[\w/]+)|Loop) \| (?P<manifold>f|[0-9-:]+)(?P<step>_[0-9]+)?(?P<levelset>_ls)?$")
    steps = {
            "_01": 1,
            "_12": 2,
            "_21": 3,
            "_10": 2,
            }
    out = []
    for id, n in zip(ids, names):
        m = regexEdge.match (n)
        if m is not None:
            d = { 'name': n, 'id': id, }
            # Compute manifold
            if m.group('manifold') == 'f': d['manifold'] = ()
            else: d['manifold'] = tuple( tuple( int(e) for e in s.split('-') ) for s in m.group('manifold').split(':') )
            if m.group('gripper') is None: # The edge is a loop
                pass
            else:
                # Compute grasp infos
                d['grasp'] = ( supervisor.grippersIdx[m.group('gripper')], supervisor.handlesIdx[m.group('handle')] )
                d['forward'] = (m.group('direction') == '>') # Warning: this is a tuple
                s = m.group('step')
                if s is None:
                    d['step'] = 3 if d['forward'] else 1
                else:
                    d['step'] = steps[s]
                if d['grasp'] in d['manifold']:
                    l = list(d['manifold'])
                    l.remove(d['grasp'])
                    d['manifold'] = tuple(l)
            out.append(d)
        else:
            print "Could not parse", n
    return out

def makeSupervisorWithFactory (robot):
    from sot_hpp import Supervisor
    from sot_hpp.factory import Factory
    from sot_hpp.tools import Manifold
    from hpp.corbaserver.manipulation import Rule

    hppclient = Client()

    grippers = [ "talos/left_gripper", "talos/right_gripper" ]
    objects = [ "box" ]
    handlesPerObjects = [ [ "box/top", "box/bottom" ], ]
    rules = [ Rule([ "talos/left_gripper", ], [ "box/top", ], True),
              Rule([ "talos/right_gripper", ], [ "box/bottom", ], True), ]

    supervisor = Supervisor (robot, hpTasks = hpTasks(robot))
    # supervisor = Supervisor (robot, hpTasks = hpTasks(robot), lpTasks = Manifold())
    factory = Factory(supervisor)
    factory.setGrippers (grippers)
    factory.setObjects (objects, handlesPerObjects, [ [] for e in objects ])
    factory.setRules (rules)
    factory.setupFrames (hppclient, robot)
    factory.generate ()
    factory.finalize (hppclient)

    supervisor.makeInitialSot ()
    return supervisor

# edges = { 'Loop | 0-0': 3, 'Loop | 0-0:1-1': 5, 'Loop | 1-1': 14, 'talos/left_gripper < box/top | 0-0:1-1': 16, 'talos/left_gripper < box/top | 0-0:1-1_10': 21, 'talos/left_gripper < box/top | 0-0:1-1_21': 20, 'talos/left_gripper > box/top | 1-1': 15, 'talos/left_gripper > box/top | 1-1_01': 18, 'talos/left_gripper > box/top | 1-1_12': 19, 'talos/right_gripper < box/bottom | 0-0:1-1': 7, 'talos/right_gripper < box/bottom | 0-0:1-1_10': 12, 'talos/right_gripper < box/bottom | 0-0:1-1_21': 11, 'talos/right_gripper > box/bottom | 0-0': 6, 'talos/right_gripper > box/bottom | 0-0_01': 9, 'talos/right_gripper > box/bottom | 0-0_12': 10 }
# supervisor = makeSupervisor(robot, edges)
supervisor = makeSupervisorWithFactory (robot)

# Add tracer
#robot.initializeTracer()
#robot.addTrace('PYRENE', 'control')
#robot.addTrace('PYRENE', 'state')
#robot.addTrace('robot_dynamic', 'Jleft-wrist')
#robot.addTrace('robot_dynamic', 'left-wrist')
#robot.startTracer()

# from dynamic_graph.ros import RosExport
from dynamic_graph_hpp.sot import RosQueuedSubscribe
re = RosQueuedSubscribe ('ros_export')

supervisor.plugTopics(re)
supervisor.setupEvents ()
supervisor.plugSot(-1)

# from dynamic_graph import writeGraph
# writeGraph('/tmp/here.dot')
