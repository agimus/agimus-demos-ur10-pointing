Put Pyrène in its half-sitting position

On talos-1m:
 - cd devel && source config.sh
 - hppcorbaserver --name estimation --single-thread&
 - hppcorbaserver
 - ipython -i script_hpp.py #Load the estimation problem
 - ipython -i estimation.py #Load the slightly different path planning problem
# Above should work
On talos-1c:
 - roslaunch agimus_demos talos_manipulate_boxes_on_pyrene_no_estimation.launch

On Nakasan:
 - cd devel-src && source config.sh
 - rosrun agimus rqt_path_exec
 - set level to 0 #Higher values will stop the robots at key steps in the demo for debugging purpose

On talos-1m
 - cd devel && source config.sh
 - roslaunch agimus_demos talos_manipulate_boxes_estimation.launch plank_of_wood_id:=3

On Nakasan:
 - cd devel-src && source config.sh
 - rosrun rviz rviz

On Nakasan:
 - cd hppdevel && source config.sh
 - gepetto-gui

