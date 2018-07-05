import sys
if not hasattr(sys, "argv"):
    sys.argv = []

class Client:
    from hpp.corbaserver import Client as BasicClient
    from hpp.corbaserver.manipulation import Client as ManipClient
    def __init__(self):
        self.basic = Client.BasicClient()
        self.manipulation = Client.ManipClient()

def makeSupervisorWithFactory (robot):
    from agimus_sot import Supervisor
    from agimus_sot.factory import Factory, Affordance
    from agimus_sot.tools import Manifold
    from hpp.corbaserver.manipulation import Rule

    holes = range(12)
    # holes = [ 11, ]
    hppclient = Client()

    grippers = [ "tiago/center", ]
    objects = [ "workspace" ]
    handlesPerObjects = [ [ "workspace/hole2_" + str(i) for i in holes ], ]
    rules = [ Rule([ "tiago/center", ], [ ".*", ], True), ]

    supervisor = Supervisor (robot)
    factory = Factory(supervisor)
    factory.setGrippers (grippers)
    factory.setObjects (objects, handlesPerObjects, [ [] for e in objects ])
    factory.setRules (rules)
    factory.setupFrames (hppclient, robot)
    # This for loop could be omitted and are here for educational purpose.
    for i in holes:
        factory.addAffordance (
                Affordance ("tiago/center", "workspace/hole2_"+str(i),
                    refOpen=(), refClose=()))
    factory.generate ()
    factory.finalize (hppclient)

    supervisor.makeInitialSot ()
    return supervisor

# q = list(robot.dynamic.position.value)
# q[0:6] = [2.4, -0.5, -3.3, 0.0, 0.0, 0.0]
# robot.dynamic.position.value = q
# robot.device.state.value = q

supervisor = makeSupervisorWithFactory (robot)

supervisor.plugTopicsToRos()
supervisor.setupEvents ()
supervisor.plugSot("")
