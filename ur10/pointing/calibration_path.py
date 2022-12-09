# In script_hpp.py, replace
#   UseAprilTagPlank = False
# by
#   UseAprilTagPlank = True
# launch script_hpp.py and in the same terminal, copy paste these lines

from calibration import Calibration, computeCameraPose

calibration = Calibration(ps, graph, factory)
calibration.nbConfigs = 100
calibration.addStateToConstraintGraph()

q_init = ri.getCurrentConfig(q_init)

calibration.generateConfigurationsAndPaths(q_init)
