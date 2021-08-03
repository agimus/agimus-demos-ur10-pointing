import estimation
import travalling_salesman
def goToArmConfiguration(robot,graph, q0):
    q_init = estimation.get_current_robot_and_cylinder_config(robot, graph, q0)
    joints = [ 'tiago/torso_lift_joint',
    'tiago/arm_1_joint',
    'tiago/arm_2_joint',
    'tiago/arm_3_joint',
    'tiago/arm_4_joint',
    'tiago/arm_5_joint',
    'tiago/arm_6_joint',
    'tiago/arm_7_joint',
    'tiago/head_1_joint',
    'tiago/head_2_joint',]
    q_goal = q_init[:]
    for j in joints:
        r = robot.rankInConfiguration[j]
        q_goal[r] = q0[r]
    res, q_goal, err = graph.applyNodeConstraints('tiago/gripper grasps driller/handle', q_goal)
    try:
        path = armPlanner.computePath(q_init,q_goal, resetRoadmap = True)
    except:
        raise RuntimeError('Failed to plan a path between ' + str(q_init) +\
            ' and ' + str(q_goal))
    return path

def gotoHandle(handle, q0, robot):
    # generate_valid_config_for_handle(handle, qinit, qguesses = [], NrandomConfig=10):
    ok, qphi2, qhi2 = generate_valid_config_for_handle(handle, q0, [], NrandomConfig=15)
    new_cluster =[]
    if ok:
            
        new_cluster.append((handle, qphi2, qhi2))
    else:
        print("Could not reach handle", handle)
        return None
    
    setRobotJointBounds("default")
    
    paths = clusters_comp.solveTSP(armPlanner, new_cluster, qhome=q0)
    path = concatenate_paths(paths)
    return path


for i in range(1000):    
    res, qres, err = graph.generateTargetConfig (e, q_init, robot.shootRandomConfig())
    if not res: continue                                                          
    print("sucess in generating random config")
    res, msg = robot.isConfigValid(qres)
    if not res:
        print (msg)
        continue
    if tiago_fov.clogged(qres, robot, tagss):
        print('clogged')
        continue
    if res: break
