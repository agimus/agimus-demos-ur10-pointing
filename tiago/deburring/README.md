# Running demo Tiago deburring
## Hardware
  - Tiago robot,
  - part P72,
  - driller,
  - laptop tatesan (login jmirabel, passwd)

## Steps to run the demo
1. plug a computer with the docker image
   `gitlab.laas.fr:4567/rob4fam/docker/tiago_deburring:4_vnc_robot`
   to Tiago with ethernet cable and select wired connection "Tiago".

2. start a docker container and a VNC client and run the commands of the next
   steps in the docket container
```
docker run -it --rm --name hpp --cap-add NET_ADMIN --net=host gitlab.laas.fr:4567/rob4fam/docker/tiago_deburring:4_vnc_robot
```
  Make sure that file `/etc/hosts` contains the following line:
```
10.68.0.1       tiago-48c
```

3. in terminal 1 launch `hppcorbaserver`
4. in terminal 2
```
  cd agimus-demos/tiago/deburring
  python2 -i travelling_salesman.py
```

5. in terminal 3
 ```
   ssh pal@tiago-wired
   stop-controllers
``` 
and check that all relevant controllers have been successfully stopped.

6. in terminal 3 (on tiago)
```
  roslaunch agimus_demos tiago_deburring_tiago.launch estimation:=false
```

7. in terminal 4
```
  ssh pal@tiago-wired
  roslaunch agimus_demos tiago_deburring_estimation.launch
```

8. in terminal 5
```
  cd agimus-demos/tiago/deburring
  roslaunch ./rviz.launch
```

9. check that the robot is well localized. If not,
  - click in rviz on button `2D pose estimate` and select an approximate pose
    with the mouse. Move the robot around its current position until the
    localization candidates converge to a reasonable value, or
  - `rosservice call /global_localization`

10. in terminal 6
```
  rosrun agimus rqt_path_execution
```

11. in terminal 7
```
  gepetto-gui
```

12. in terminal 2
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
