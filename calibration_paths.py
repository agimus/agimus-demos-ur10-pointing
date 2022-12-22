# In script_hpp.py, replace
#   UseAprilTagPlank = False
# by
#   UseAprilTagPlank = True
# launch script_hpp.py and in the same terminal, copy paste these lines

from calibration import Calibration, computeCameraPose

calibration = Calibration(ps, graph, factory)
calibration.nbConfigs = 30
calibration.addStateToConstraintGraph()
calibration.generateConfigurationsAndPaths(q0, filename="./data/calib-configs.csv")
