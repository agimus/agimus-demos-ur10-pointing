from dynamic_graph.sot.universal_robot import UniversalRobot
import agimus_sot.factory
from agimus_sot.react import TaskFactory

agimus_sot.factory.logging.basicConfig(filename = '/tmp/supervisor.log',
                                       level=agimus_sot.factory.logging.DEBUG)
simulateTorqueFeedbackForEndEffector = False

class After(object):
    def addSignal(self, signal):
        pass

def makeSupervisorWithFactory(robot):
    from agimus_sot import Supervisor
    from agimus_sot.factory import Factory, Affordance
    from agimus_sot.srdf_parser import parse_srdf, attach_all_to_link
    import pinocchio
    import rospy

    if not hasattr(robot, "camera_frame"):
        robot.camera_frame = "xtion_optical_frame"

    srdf = {}
    # retrieve objects from ros param
    demoDict = rospy.get_param("/demo")
    robotDict = demoDict["robots"]
    if len(robotDict) != 1:
        raise RuntimeError("One and only one robot is supported for now.")
    objectDict = demoDict["objects"]
    objects = list(objectDict.keys())
    # parse robot and object srdf files
    srdfDict = dict()
    for r, data in robotDict.items():
        srdfDict[r] = parse_srdf(srdf = data["srdf"]["file"],
                                 packageName = data["srdf"]["package"],
                                 prefix=r)
    for o, data in objectDict.items():
        srdfDict[o] = parse_srdf(srdf = data["srdf"]["file"],
                                 packageName = data["srdf"]["package"],
                                 prefix=o)

    grippers = list(demoDict["grippers"])
    handlesPerObjects = list()
    contactPerObjects = list()
    for o in objects:
        handlesPerObjects.append(sorted(list(srdfDict[o]["handles"].keys())))
        contactPerObjects.append(sorted(list(srdfDict[o]["contacts"].keys())))

    for w in ["grippers", "handles","contacts"]:
        srdf[w] = dict()
        for k, data in srdfDict.items():
            srdf[w].update(data[w])

    supervisor = Supervisor(robot, prefix=robotDict.keys()[0])
    factory = Factory(supervisor)
    factory.tasks = TaskFactory(factory)
    factory.parameters["period"] = 0.01  # TODO soon: robot.getTimeStep()
    factory.parameters["simulateTorqueFeedback"] = simulateTorqueFeedbackForEndEffector
    factory.parameters["addTracerToAdmittanceController"] = False
    # factory.parameters["addTimerToSotControl"] = True
    factory.setGrippers(grippers)
    factory.setObjects(objects, handlesPerObjects, contactPerObjects)

    from hpp.corbaserver.manipulation import Rule
    factory.setupFrames(srdf["grippers"], srdf["handles"], robot)
    for k in handlesPerObjects[0]:
        factory.handleFrames[k].hasVisualTag = True
    factory.generate()

    supervisor.makeInitialSot()
    return supervisor, factory

robot = UniversalRobot('UR10')
robot.device.after = After()

supervisor, factory = makeSupervisorWithFactory(robot)

from dynamic_graph import writeGraph
writeGraph('/tmp/sot.dot')
