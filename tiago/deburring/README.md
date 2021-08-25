# Running demo Tiago deburring
## Hardware
  - Tiago robot,
  - part P72,
  - driller,
  - laptop tatesan (login jmirabel, passwd)

## Steps to run the demo

The demonstration can be run in simulation in Gazebo. Specific steps for that are prefixed by *simulation:*.

1. plug a computer with the docker image
   `gitlab.laas.fr:4567/rob4fam/docker/tiago_deburring:1_vnc_robot`
   to Tiago with ethernet cable and select wired connection "Tiago".

2. start a docker container and a VNC client and run the commands of the next
   steps in the docket container
```
docker run -it --rm --name hpp --cap-add NET_ADMIN --net=host gitlab.laas.fr:4567/rob4fam/docker/tiago_deburring:1_vnc_robot
```
  Make sure that file `/etc/hosts` contains the following line:
```
10.68.0.1       tiago-48c
```

3. *simulation:* in terminal 0
```bash
  roslaunch agimus_demos tiago_deburring_simulation.launch
```

4. in terminal 1 launch `hppcorbaserver`
5. in terminal 2
```
  cd agimus-demos/tiago/deburring
  python2 -i travelling_salesman.py
```

6. *simulation:* in terminal 3
```bash
  cd agimus-demos/tiago/deburring
  python stop-controllers.py
```

6. in terminal 3
```bash
   ssh pal@tiago-wired
   stop-controllers
```
and check that all relevant controllers have been successfully stopped.

7. *simulation:* in terminal 3
```
  roslaunch agimus_demos tiago_deburring_demo.launch estimation:=false simulation:=true
```

7. in terminal 3
```
  roslaunch agimus_demos tiago_deburring_demo.launch estimation:=false
```

8. in terminal 4
```
  ssh pal@tiago-wired
  roslaunch agimus_demos tiago_deburring_estimation.launch
```

9. in terminal 5
```
  cd agimus-demos/tiago/deburring
  roslaunch ./rviz.launch
```

10. *simulation:* in terminal 6
```python
  python initial_pose.py
```
interrupt the script when the robot is correctly localized in rviz

10. check that the robot is well localized. If not,
  - click in rviz on button `2D pose estimate` and select an approximate pose
    with the mouse. Move the robot around its current position until the
    localization candidates converge to a reasonable value, or
  - `rosservice call /global_localization`

11. in terminal 6
```
  rosrun agimus rqt_path_execution
```

12. in terminal 7
```
  gepetto-gui
```

13. in terminal 2
```python
bpath = compute_base_path_to_cluster_init(0)
ps.hppcorba.problem.addPath(bpath) # Returns the path ID in HPP
```
You can visualize the trajectory after refreshing gepetto-gui. If you are
satisfied, you can execute the trajectory using the rqt_path_execution window.
The robot starts closing the gripper. Don't forget to put the driller in the
hand.

You may have to recompute the clusters if HPP fails to reach the handles. After
having recomputed the clusters, you must move the base again, as above
(step 12).
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


Instead of step 9, you can run the script                                                                                                                     
```python
python initial_pose.py
```
That will send the initial pose of the robot to the navigation stack.
