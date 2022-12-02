This files reports on experiments run on the UR10e robot

Investigations on random crashes of "ur10e_bringup.launch".
-----------------------------------------------------------

Test 1
======

Start
  + roscontrol, agimus, hppcorbaserver,
  + plan several paths and execute them.
No crash observed.

Test 2
======

Start
 + roscontrol, agimus, hppcorbaserver,
 + rosnode for realsense sensor,
 + plan several paths and execute them.
No crash observed.

Test 3
======

Start
  + roscontrol, agimus, hppcorbaserver,
  + rosnode for realsense sensor,
  + start react_inria localization,
  + plan several paths and execute them.
No crash observed.

Observation
===========

During one of the motion, the robot went in autocollision because of the
penetration parameter (1 cm) of the path validation algorithm.
