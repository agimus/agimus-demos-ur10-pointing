# Running demo Tiago deburring
## Hardware
  - Tiago robot,
  - part P72,
  - driller,
  - laptop tatesan (login jmirabel, passwd)

## Steps to run de demo
1. plug tatesan to Tiago with ethernet cable and select wired connection "Tiago".

#### Note: in each terminal, always run
```
  source ~/devel/tiago/set_env_for_tiago
```
2. in terminal 1 launch `hppcorbaserver`
3. in terminal 2
```
  cd agimus-demos/tiago/deburring
  python2 -i travelling_salesman.py
```
4. in terminal 3
```
  cd agimus-demos/tiago/deburring
  roslaunch ./rviz.launch
```

5. check that the robot is well localized. If not, click in rviz on button `2D pose estimate` and select an approximate pose with the mouse. Move the robot around its current position until the localization candidates converge to a reasonable value.

6. in terminal 4
 ```
   ssh tiago-wired
   stop-controllers
``` 
and check that all relevant controllers have been successfully stopped.

7. in terminal 4
```
  roslaunch agimus_demos tiago_deburring_tiago.launch estimation:=true
```

8. in terminal 5
```
  rosrun agimus rqt_path_execution
```

9. in terminal 2
```python
bpath = compute_base_path_to_cluster_init(0)
ps.hppcorba.problem.addPath(bpath) # Returns the path ID in HPP
```
You can visualize the trajectory after refreshing gepetto-gui. If you are satisfied, you can execute the trajectory using the rqt_path_execution window. The robot starts closing the gripper. Don't forget to put the driller in the hand.

You may have to recompute the clusters if HPP fails to reach the handles. After having recomputed the clusters, you must move the base again, as above (step 9).
```python
clusters = recompute_clusters()
```
You can now compute a motion of the arm to reach the holes.
```python
ncluster, apath = compute_path_for_cluster(0)
# ncluster contains the new cluster (reached holes, with the updated configs)
# apath is the wholebody trajectory
apath_id = ps.hppcorba.problem.addPath(apath)
ps.optimizePath(apath_id)
# Visualize the last path in gepetto-gui
```
