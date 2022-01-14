# Running react_inria localization node

To provide the initial pose of the part in simulation

```bash
roslaunch ./react_learning.launch use_sim_time:=true
```
 To run the localization node in simulation

```
roslaunch ./react_localizer.launch use_wMo_filename:=$HOME/.ros/transform_wMo.yaml use_oMo_filename:=$HOME/.ros/transform_oMo.yaml use_aligned_depth:=false use_sim_time:=true
```
