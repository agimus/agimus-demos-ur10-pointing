q1 = ps.configAtParam (0, ps.pathLength (1))
q2 = q1 [::]
rank = robot.rankInConfiguration["box/root_joint"]
q2 [rank + 3 : rank + 7] = (
    Quaternion([0, 1, 0, 0]) * Quaternion(q_init[rank + 3 : rank + 7])).\
    toTuple()
ps.resetGoalConfigs ()
ps.setInitialConfig (q1)
ps.addGoalConfig (q2)
ps.setMaxIterPathPlanning (1000)
