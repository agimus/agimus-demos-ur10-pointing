Installation instructions
=========================

Preliminary remarks
-------------------

In this page, we propose to install the software in two different profiles.
  * ```HPP``` in one profile,
  * ```Agimus``` and the Stack of Tasks (```SoT```) in the second profile.

By "profile", we mean a set of values for environment variables that are used to find the right piece of software. Mainly
```
PATH
LD_LIBRARY_PATH
PYTHONPATH
PKG_CONFIG_PATH
ROS_PACKAGE_PATH
```

The reasons for this choice that may increase the level of complexity in the installation procedure are
  * ```SoT``` and ```HPP``` may use different versions of ```pinocchio``` and ```hpp-fcl```
  * ```Agimus``` compiles ```hpp-corbaserver``` with option ```client-only``` and therefore installs a different ```libhpp-corbaserver.So``` library.

When opening a terminal, the user can switch from the default profile defined in ```.bashrc``` file to the desired profile using command ```change_develconfig``` installed by ```hpp-tools```.

Choosing a root directory
-------------------------

The first thing to do is to choose a root directory where the source code will be compiled and installed. In the following instructions, this directory is ```${HOME}/devel/pyrene```. Add the following lines in your ```.bashrc```:
```
if [[ -z "$DEVEL_CONFIG" ]]; then
  export DEVEL_CONFIG=hpp
fi

echo Configuration $DEVEL_CONFIG

export DEVEL_HPP_DIR=${HOME}/devel/pyrene/${DEVEL_CONFIG}
if [ -d $DEVEL_HPP_DIR ] ; then 
  if [ -f $DEVEL_HPP_DIR/config.sh ]; then
    source $DEVEL_HPP_DIR/config.sh
  fi
fi
```

ROS, Stack of Tasks and Talos software
--------------------------------------

Follow the instructions on the Pyrene Installation Software page.

Compilation options
-------------------

If you install from source, it may be useful to tune the compilation options for the target system: either the control or multimedia computer. To know what compilation flags should be enable for a specific computer, you can run the following command on the target computer.
```
gcc -march=native -E -v - </dev/null 2>&1 | grep cc1
```
The ```-dM``` option may be useful too.

Good sources of information:
- https://stackoverflow.com/questions/5470257/how-to-see-which-flags-march-native-will-activate

HPP
---

**From source**

Go to the [following page](https://humanoid-path-planner.github.io/hpp-doc/download.html). Choose "Stable" or "Development" option and follow instructions in Section "Source installation...". Before compiling, add the following lines at the end of the Makefile (step 5).
```
hpp-fcl.install:
	@echo "Do not install hpp-fcl"

pinocchio.install:
	@echo "Do not install pinocchio"
```

Note that
  * ```DEVEL_HPP_DIR``` is already set (be sure to open a new terminal so that ```.bashrc``` is executed).

Install hpp-tools in order to be able to switch to another profile.
```
cd $DEVEL_HPP_DIR/src
make hpp-tools.install
```

Agimus
------

**Additional Ubuntu-16.04 packages**

```
sudo apt-get install ros-kinetic-dynamic-robot-state-publisher ros-kinetic-smach-ros ros-kinetic-qt-gui-py-common robotpkg-gerard-bauzil ros-kinetic-roslint ros-kinetic-visp-bridge ros-kinetic-tf2-geometry-msgs
```

**Additional Ubuntu-18.04 packages**

```
sudo apt install libbullet-dev libcv-bridge-dev libimage-transport-dev libtf2-bullet-dev python3-empy robotpkg-ros-lint robotpkg-ros-rqt robotpkg-ros-rviz ros-melodic-realtime-tools ros-melodic-rqt ros-melodic-dynamic-robot-state-publisher ros-melodic-qt-gui ros-melodic-qt-gui-py-common ros-melodic-roslint ros-melodic-smach-ros ros-melodic-tf2-geometry-msgs ros-melodic-visp-bridge
```

**From source**

Agimus will be installed in profile "agimus". To switch profile, open a terminal and type
```
change_develconfig agimus
```
The terminal should display the following message
```
Unsetting variables INSTALL_HPP_DIR ROS_ROOT ROS_PACKAGE_PATH ROS_MASTER_URI DEVEL_CONFIG ROS_VERSION LD_LIBRARY_PATH PATH PWD ROSLISP_PACKAGE_DIRECTORIES ROS_DISTRO DEVEL_HPP_DIR PYTHONPATH PKG_CONFIG_PATH CMAKE_PREFIX_PATH ROS_ETC_DIR
Profile agimus
```

```
mkdir -p $DEVEL_HPP_DIR/src
wget -O $DEVEL_HPP_DIR/src/Makefile https://raw.githubusercontent.com/agimus/agimus/master/doc/Makefile
wget -O $DEVEL_HPP_DIR/config.sh https://raw.githubusercontent.com/humanoid-path-planner/hpp-doc/devel/doc/config/ubuntu-16.04-kinetic.sh
source $DEVEL_HPP_DIR/config.sh
cd $DEVEL_HPP_DIR/src
make bootstrap
make all
make agimus-demos.install
```

The Stack of Tasks stores plugin in a non standard directory. To make it able to find those plugins, add the following line to ```$DEVEL_HPP_DIR/config.sh```:
```
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROBOTPKG/lib/plugin
```

Simulation
==========

Using geometric_simu
--------------------

  * In terminal 1 (profile "hpp"), run ```hppcorbaserver```,
  * in terminal 2 (profile "agimus"), run ```script_hpp.py``` for path planning
```
cd $DEVEL_HPP_DIR/src/agimus-demos/talos/manipulate_boxes
python -i script_hpp.py
```
  * in terminal 3 (profile "agimus"), run ```script_hpp.py``` for simulation
```
cd $DEVEL_HPP_DIR/src/agimus-demos/talos/manipulate_boxes
python script_hpp.py --context simulation
```
  * in terminal 3 (profile "agimus"), run ```script_hpp.py``` for estimation
```
python script_hpp.py --context estimation
```
  * in terminal 3 again, launch the simulation
```
roslaunch agimus_demos talos_manipulate_boxes_geometric_simu.launch
```
  * in terminal 4 (profile "agimus"), launch the estimation
```
roslaunch agimus_demos talos_manipulate_boxes_estimation.launch state_estimation:=true
```
  * in python terminal 2 type
```
from plan_path import planPath
planPath (ps)
```

**Executing the path**

In the window "agimus_Path execution - rqt", click on "Execute path" after selecting the right path id (2 for leaning forward, 5 for grasping motion - to be checked in gepetto-gui)


