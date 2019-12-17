from hpp import Quaternion
# Plan a path to flip the box starting at the end of pathId.
def planPath (ps, pathId = None):
    if pathId is None: pathId = ps.numberPaths () - 1
    q1 = ps.configAtParam (0, ps.pathLength (pathId))
    q2 = q1 [::]
    rank = ps.robot.rankInConfiguration["box/root_joint"]
    q2 [rank + 3 : rank + 7] = (
        Quaternion([0, 1, 0, 0]) * Quaternion(q1[rank + 3 : rank + 7])).\
        toTuple()
    ps.resetGoalConfigs ()
    ps.setInitialConfig (q1)
    ps.addGoalConfig (q2)
    ps.setMaxIterPathPlanning (1000)
    ps.solve ()
