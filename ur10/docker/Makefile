#
# Copyright (c) 2014 CNRS-LAAS
# Author: Florent Lamiraux
#

FCL_REPO=https://github.com/flexible-collision-library
LAAS_REPO=https://github.com/laas
HPP_REPO=https://github.com/humanoid-path-planner
SOT_REPO=https://github.com/stack-of-tasks
GEPETTO_REPO=https://github.com/Gepetto
LOCO3D_REPO=https://github.com/loco-3d
AGIMUS_REPO=https://github.com/agimus
INTEL_REPO=https://github.com/IntelRealSense
FLORENT_REPO=https://github.com/florent-lamiraux
PAL_REPO=https://github.com/pal-robotics
GEPETTO_GITLAB_REPO=https://gitlab.laas.fr/gepetto
ROB4FAM_GITLAB_REPO=https://gitlab.laas.fr/rob4fam
INRIA_GITLAB_REPO=https://gitlab.inria.fr/rainbow
PYTHON_EXECUTABLE=/usr/bin/python3

SRC_DIR=${DEVEL_HPP_DIR}/src
ifndef INSTALL_HPP_DIR
INSTALL_HPP_DIR=${DEVEL_HPP_DIR}/install
endif

BUILD_TYPE?=Release
BUILD_TESTING?=ON
ifeq (${BUILD_TYPE},Debug)
  BUILD_FOLDER=build
else
  BUILD_FOLDER=build-rel
  BUILD_TESTING=OFF
endif

OSG_PACKAGE=osg-dae
#OSG_PACKAGE=OpenSceneGraph-3.4.0

WGET=wget --quiet
UNZIP=unzip -qq
TAR=tar
GIT_QUIET=--quiet
# Qt version should be either 4 or 5
QT_VERSION=5
INSTALL_DOCUMENTATION=ON

##################################
# {{{ Dependencies

pinocchio_branch=v2.6.8
pinocchio_repository=${SOT_REPO}
pinocchio_extra_flags= -DBUILD_PYTHON_INTERFACE=ON -DBUILD_UNIT_TESTS=OFF -DBUILD_WITH_COLLISION_SUPPORT=ON -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

hpp-template-corba_branch=devel
hpp-template-corba_repository=${HPP_REPO}
hpp-template-corba_extra_flags= -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}
# }}}
##################################
# {{{ Packages supporting HPP_VERSION

# Either a version tag (e.g. v4.3.0), stable or devel
HPP_VERSION=devel
HPP_EXTRA_FLAGS= -DBUILD_TESTING=${BUILD_TESTING}

hpp-util_branch=${HPP_VERSION}
hpp-util_repository=${HPP_REPO}

hpp-fcl_branch=react
hpp-fcl_repository=${FLORENT_REPO}
hpp-fcl_extra_flags= -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=ON -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE} ${HPP_EXTRA_FLAGS}

hpp-statistics_branch=${HPP_VERSION}
hpp-statistics_repository=${HPP_REPO}

hpp-pinocchio_branch=${HPP_VERSION}
hpp-pinocchio_repository=${HPP_REPO}
hpp-pinocchio_extra_flags=${HPP_EXTRA_FLAGS}

hpp-constraints_branch=${HPP_VERSION}
hpp-constraints_repository=${HPP_REPO}
hpp-constraints_extra_flags=${HPP_EXTRA_FLAGS} -DUSE_QPOASES=OFF

hpp-core_branch=${HPP_VERSION}
hpp-core_repository=${HPP_REPO}
hpp-core_extra_flags=${HPP_EXTRA_FLAGS}

hpp-corbaserver_branch=${HPP_VERSION}
hpp-corbaserver_repository=${HPP_REPO}
hpp-corbaserver_extra_flags= -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

hpp-doc_branch=${HPP_VERSION}
hpp-doc_repository=${HPP_REPO}

hpp-manipulation_branch=${HPP_VERSION}
hpp-manipulation_repository=${HPP_REPO}
hpp-manipulation_extra_flags=

hpp-manipulation-urdf_branch=${HPP_VERSION}
hpp-manipulation-urdf_repository=${HPP_REPO}

hpp-manipulation-corba_branch=${HPP_VERSION}
hpp-manipulation-corba_repository=${HPP_REPO}
hpp-manipulation-corba_extra_flags= -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

hpp_tutorial_branch=${HPP_VERSION}
hpp_tutorial_repository=${HPP_REPO}

hpp-gepetto-viewer_branch=${HPP_VERSION}
hpp-gepetto-viewer_repository=${HPP_REPO}
hpp-gepetto-viewer_extra_flags= -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

hpp-plot_branch=${HPP_VERSION}
hpp-plot_repository=${HPP_REPO}

hpp-gui_branch=${HPP_VERSION}
hpp-gui_repository=${HPP_REPO}

# }}}
##################################
# {{{ Robot specific package + test packages

example-robot-data_branch=devel
example-robot-data_repository=${GEPETTO_REPO}
example-robot-data_extra_flags= -DBUILD_PYTHON_INTERFACE=OFF

hpp_benchmark_branch=devel
hpp_benchmark_repository=${HPP_REPO}

hpp-environments_branch=${HPP_VERSION}
hpp-environments_repository=${HPP_REPO}

universal_robot_branch=calibration_devel
universal_robot_repository=https://github.com/fmauch

hpp-universal-robot_branch=${HPP_VERSION}
hpp-universal-robot_repository=${HPP_REPO}

hpp-baxter_branch=${HPP_VERSION}
hpp-baxter_repository=${HPP_REPO}

hpp_romeo_branch=${HPP_VERSION}
hpp_romeo_repository=${HPP_REPO}

Universal_Robots_Client_Library_repository=https://github.com/UniversalRobots
Universal_Robots_Client_Library_branch=boost
Universal_Robots_Client_Library_extra_flags=

Universal_Robots_ROS_Driver_repository=https://github.com/UniversalRobots
Universal_Robots_ROS_Driver_branch=master
Universal_Robots_ROS_Driver_extra_flags=

sot-universal-robot_repository=${SOT_REPO}
sot-universal-robot_branch=master
sot-universal-robot_extra_flags=-DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

# }}}
##################################
# {{{ Packages for rbprm

hpp-affordance_branch=${HPP_VERSION}
hpp-affordance_repository=${HPP_REPO}
hpp-affordance_extra_flags=${HPP_EXTRA_FLAGS}

hpp-affordance-corba_branch=${HPP_VERSION}
hpp-affordance-corba_repository=${HPP_REPO}
hpp-affordance-corba_extra_flags=${HPP_EXTRA_FLAGS}

anymal-rbprm_branch=${HPP_VERSION}
anymal-rbprm_repository=${HPP_REPO}

hyq-rbprm_branch=${HPP_VERSION}
hyq-rbprm_repository=${HPP_REPO}

simple-humanoid-rbprm_branch=${HPP_VERSION}
simple-humanoid-rbprm_repository=${HPP_REPO}

talos-rbprm_branch=${HPP_VERSION}
talos-rbprm_repository=${HPP_REPO}

hpp-rbprm_branch=${HPP_VERSION}
hpp-rbprm_repository=${HPP_REPO}
hpp-rbprm_extra_flags=${HPP_EXTRA_FLAGS}

hpp-rbprm-robot-data_branch=${HPP_VERSION}
hpp-rbprm-robot-data_repository=${HPP_REPO}

hpp-rbprm-corba_branch=${HPP_VERSION}
hpp-rbprm-corba_repository=${HPP_REPO}
hpp-rbprm-corba_extra_flags=${HPP_EXTRA_FLAGS}

hpp-centroidal-dynamics_branch=${HPP_VERSION}
hpp-centroidal-dynamics_repository=${HPP_REPO}

hpp-bezier-com-traj_branch=${HPP_VERSION}
hpp-bezier-com-traj_repository=${HPP_REPO}
hpp-bezier-com-traj_extra_flags= -DBUILD_PYTHON_INTERFACE=ON

ndcurves_branch=${HPP_VERSION}
ndcurves_repository=${LOCO3D_REPO}
ndcurves_extra_flags= -DBUILD_PYTHON_INTERFACE=ON

eigenpy_branch=v2.7.13
eigenpy_repository=${SOT_REPO}
eigenpy_extra_flags= -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

# }}}
##################################
# {{{ Packages for gepetto-gui

collada-dom_branch=master
collada-dom_repository=${HPP_REPO}
collada-dom_extra_flags=-DBUILD_SHARED_LIBS=TRUE -DOPT_COLLADA15=FALSE

osg-dae_branch=master
osg-dae_repository=${GEPETTO_REPO}
osg-dae_extra_flags= -DCOLLADA_DYNAMIC_LIBRARY=${INSTALL_HPP_DIR}/lib/libcollada14dom.so -DCOLLADA_INCLUDE_DIR=${INSTALL_HPP_DIR}/include/collada-dom
OpenSceneGraph-3.4.0_extra_flags= -DDESIRED_QT_VERSION=${QT_VERSION} -DCOLLADA_DYNAMIC_LIBRARY=${INSTALL_HPP_DIR}/lib/libcollada14dom.so -DCOLLADA_INCLUDE_DIR=${INSTALL_HPP_DIR}/include/collada-dom -DLIB_POSTFIX=""

gepetto-viewer_branch=${HPP_VERSION}
gepetto-viewer_repository=${GEPETTO_REPO}
ifeq (${QT_VERSION}, 5)
	gepetto-viewer_extra_flags= -DPROJECT_USE_QT4=OFF
else
	gepetto-viewer_extra_flags= -DPROJECT_USE_QT4=ON
endif

gepetto-viewer-corba_branch=${HPP_VERSION}
gepetto-viewer-corba_repository=${GEPETTO_REPO}
gepetto-viewer-corba_extra_flags= -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

qgv_branch=devel
qgv_repository=${HPP_REPO}
ifeq (${QT_VERSION}, 5)
	qgv_extra_flags=-DBINDINGS_QT5=ON -DBINDINGS_QT4=OFF
else
	qgv_extra_flags=-DBINDINGS_QT5=OFF -DBINDINGS_QT4=ON
endif

hpp-tools_branch=${HPP_VERSION}
hpp-tools_repository=${HPP_REPO}
hpp-tools_extra_flags=

# }}}
##################################
# {{{ Packages for Agimus

agimus_branch=master
agimus_repository=${AGIMUS_REPO}
agimus_extra_flags= -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

agimus-sot_branch=devel
agimus-sot_repository=${AGIMUS_REPO}
agimus-sot_extra_flags= -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

agimus_sot_msgs_branch=master
agimus_sot_msgs_repository=${AGIMUS_REPO}

agimus-hpp_branch=devel
agimus-hpp_repository=${AGIMUS_REPO}
agimus-hpp_extra_flags=-DBUILD_ROS_INTERFACE=ON -DCLIENT_TO_GEPETTO_VIEWER=ON \
	-DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

agimus-vision_branch=devel
agimus-vision_repository=${AGIMUS_REPO}

agimus-demos_branch=devel
agimus-demos_repository=${AGIMUS_REPO}
agimus-demo_extraflags= -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

agimus-doc_branch=devel
agimus-doc_repository=${AGIMUS_REPO}

gerard-bauzil_branch=devel
gerard-bauzil_repository=${GEPETTO_GITLAB_REPO}

dynamic-graph_branch=devel
dynamic-graph_repository=${SOT_REPO}
dynamic-graph_extra_flags=-DSUFFIX_SO_VERSION=OFF

fake-ros_branch=master
fake-ros_repository=${FLORENT_REPO}
fake-ros_extra_flags=

# }}}
##################################
# {{{ Packages for SoT

dynamic-graph-python_branch=devel
dynamic-graph-python_repository=${SOT_REPO}
dynamic-graph-python_extra_flags=-DSUFFIX_SO_VERSION=OFF -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

dynamic_graph_bridge_branch=devel
dynamic_graph_bridge_repository=${SOT_REPO}

dynamic_graph_bridge_msgs_branch=master
dynamic_graph_bridge_msgs_repository=${SOT_REPO}

sot-core_branch=devel
sot-core_repository=${SOT_REPO}
sot-core_extra_flags=-DSUFFIX_SO_VERSION=OFF -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

sot-tools_branch=devel
sot-tools_repository=${SOT_REPO}
sot-tools_extra_flags=-DSUFFIX_SO_VERSION=OFF -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

sot-doc_repository=${SOT_REPO}
sot-doc_branch=master

sot-dynamic-pinocchio_branch=devel
sot-dynamic-pinocchio_repository=${SOT_REPO}
sot-dynamic-pinocchio_extra_flags=-DSUFFIX_SO_VERSION=OFF -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}

roscontrol_sot_branch=devel
roscontrol_sot_repository=${SOT_REPO}

pal_msgs_repository=${PAL_REPO}
pal_msgs_branch=indigo-devel
pal_msgs_extra_flags=

pal_hardware_interfaces_repository=${PAL_REPO}
pal_hardware_interfaces_branch=indigo-devel
pal_hardware_interfaces_extra_flags=

# }}}
##################################
# {{{ Packages for vision

rviz_camera_stream_branch=master
rviz_camera_stream_repository=${AGIMUS_REPO}

rviz_lighting_branch=master
rviz_lighting_repository=${AGIMUS_REPO}

librealsense_repository=${INTEL_REPO}
librealsense_branch=master
librealsense_extra_flags=-DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false -DBUILD_GLSL_EXTENSIONS=false -DCMAKE_BUILD_TYPE=Release

# realsense-ros_repository=${FLORENT_REPO}
# realsense-ros_branch=noetic-devel
# realsense-ros_extraflags=

realsense_gazebo_plugin_repository=${PAL_REPO}
realsense_gazebo_plugin_branch=melodic-devel
realsense_gazebo_plugin_extra_flags=

opencv_repository=https://github.com/opencv
opencv_branch=master
opencv_extra_flags=-DBUILD_PERF_TESTS=Off -DBUILD_TESTS=Off -DBUILD_EXAMPLES=Off -DBUILD_opencv_apps=Off -DBUILD_opencv_calib3d=On -DBUILD_opencv_core=On -DBUILD_opencv_dnn=Off -DBUILD_opencv_features2d=On -DBUILD_opencv_flann=On -DBUILD_opencv_highgui=On -DBUILD_opencv_imgcodecs=On -DBUILD_opencv_imgproc=On -DBUILD_opencv_java_bindings_generator=Off -DBUILD_opencv_js=Off -DBUILD_opencv_ml=Off -DBUILD_opencv_objdetect=Off -DBUILD_opencv_photo=Off -DBUILD_opencv_python2=Off -DBUILD_opencv_python3=Off -DBUILD_opencv_python_bindings_generator=Off -DBUILD_opencv_python_tests=Off -DBUILD_opencv_shape=Off -DBUILD_opencv_stitching=Off -DBUILD_opencv_superres=Off -DBUILD_opencv_ts=Off -DBUILD_opencv_video=On -DBUILD_opencv_videoio=On -DBUILD_opencv_videostab=Off -DBUILD_opencv_world=Off -DWITH_JASPER=Off -DWITH_ITT=Off -DWITH_JPEG=On -DWITH_WEBP=Off -DWITH_PNG=On -DWITH_TIFF=Off -DWITH_OPENEXR=Off -DWITH_OPENCL=Off-DWITH_IPP=Off -DWITH_PROTOBUF=Off -DWITH_GTK_2_X=On -DCMAKE_BUILD_TYPE=Release

visp_repository=https://github.com/lagadic
visp_branch=master
visp_extra_flags=-DBUILD_JAVA=OFF -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_DEMOS=OFF -DBUILD_TUTORIALS=OFF-DBUILD_APPS=OFF -DBUILD_DEPRECATED_FUNCTIONS=OFF -DCMAKE_BUILD_TYPE=Release

rgbd_launch_repository=https://github.com/ros-drivers
rgbd_launch_branch=indigo-devel
rgbd_launch_extra_flags=

vision_opencv_repository=https://github.com/ros-perception
vision_opencv_branch=noetic
vision_opencv_extra_flags=

vision_visp_repository=https://github.com/lagadic
vision_visp_branch=master
vision_visp_extra_flags=

image_pipeline_repository= https://github.com/ros-perception
image_pipeline_branch=noetic
image_pipeline_extra_flags=

react_inria_repository=${ROB4FAM_GITLAB_REPO}
react_inria_branch=rob4fam
react_inria_extra_flags=

# }}}
##################################
# {{{ High-level targets

all: realsense_gazebo_plugin.install\
	rgbd_launch.install visp_auto_tracker.install \
	visp_camera_calibration.install visp_hand2eye_calibration.install \
	hpp-manipulation-corba.install hpp-gepetto-viewer.install \
	hpp-plot.install hpp-gui.install sot-universal-robot.install \
	roscontrol_sot.install gerard-bauzil.install agimus-sot.install \
	agimus-hpp.install agimus.install \
	agimus-demos.install rviz_camera_stream.install rviz_lighting.install \
	Universal_Robots_ROS_Driver.install

# For test on gepgitlab, install robot packages first
test-ci: example-robot-data.install  hpp-environments.install \
	hpp-baxter.install hpp-wholebody-step.install
	${MAKE} hpp_tutorial.install hpp-gepetto-viewer.install hpp-rbprm-corba.install \
	hpp-universal-robot.install && \
	${MAKE} hpp-doc.install

# For benchmark, install robot packages first
benchmark: example-robot-data.install hpp-environments.install
	${MAKE} hpp_tutorial.install hpp-gepetto-viewer.install; \
	${MAKE} hpp-baxter.install hpp_romeo.install \
	hpp-universal-robot.install hpp-plot.install hpp-gui.install; \
	${MAKE} hpp_benchmark.checkout; \
	${MAKE} hpp-doc.install

rbprm: hpp-rbprm-corba.install hpp-gepetto-viewer.install
	${MAKE} hpp-doc.install


# }}}
##################################
# {{{ Dependencies declaration

hpp-doc.configure.dep: hpp-doc.checkout
hpp-fcl.configure.dep: eigenpy.install hpp-fcl.checkout
hpp-walkgen.configure.dep: hpp-util.install hpp-core.install \
	hpp-walkgen.checkout
hpp-util.configure.dep: hpp-util.checkout
hpp-model-urdf.configure.dep: hpp-model.install hpp-model-urdf.checkout
pinocchio.configure.dep: hpp-fcl.install pinocchio.checkout
hpp-pinocchio.configure.dep: pinocchio.install hpp-util.install \
	hpp-pinocchio.checkout
hpp-statistics.configure.dep: hpp-util.install hpp-statistics.checkout
hpp-core.configure.dep: example-robot-data.install hpp-constraints.install \
	hpp-statistics.install hpp-core.checkout
hpp-constraints.configure.dep: hpp-pinocchio.install hpp-statistics.install \
	hpp-environments.install hpp-constraints.checkout
hpp-wholebody-step.configure.dep: hpp-constraints.install hpp-walkgen.install \
	hpp-wholebody-step.checkout
hpp-manipulation.configure.dep: hpp-core.install hpp-constraints.install \
	hpp-manipulation.checkout
hpp-manipulation-corba.configure.dep: hpp-manipulation-urdf.install \
	hpp-manipulation.install hpp-corbaserver.install \
	hpp-template-corba.install hpp-manipulation-corba.checkout
hpp-plot.configure.dep: hpp-corbaserver.install hpp-manipulation-corba.install \
	gepetto-viewer-corba.install hpp-plot.checkout
hpp-manipulation-urdf.configure.dep:hpp-manipulation.install \
	hpp-manipulation-urdf.checkout
hpp-corbaserver.configure.dep: hpp-core.install hpp-template-corba.install \
	hpp-constraints.install hpp-corbaserver.checkout
hpp-template-corba.configure.dep: hpp-util.install hpp-template-corba.checkout
qgv.configure.dep: qgv.checkout
robot_model_py.configure.dep: robot_model_py.checkout
robot_capsule_urdf.configure.dep: robot_model_py.install \
	robot_capsule_urdf.checkout
test-hpp.configure.dep: hpp-wholebody-step.install \
	hpp-gepetto-viewer.install test-hpp.checkout
hpp_tutorial.configure.dep: hpp-gepetto-viewer.install \
	hpp-manipulation-corba.install hpp_tutorial.checkout
hpp_benchmark.configure.dep: hpp_tutorial.install hpp_benchmark.checkout
collada-dom.configure.dep: collada-dom.checkout
osg-dae.configure.dep: collada-dom.install \
	osg-dae.checkout
OpenSceneGraph-3.4.0.configure.dep: collada-dom.install \
	OpenSceneGraph-3.4.0.checkout
gepetto-viewer.configure.dep: gepetto-viewer.checkout
gepetto-viewer-corba.configure.dep: gepetto-viewer.install \
	gepetto-viewer-corba.checkout
hpp-gepetto-viewer.configure.dep: gepetto-viewer-corba.install \
	hpp-corbaserver.install \
	hpp-gepetto-viewer.checkout
hpp-gui.configure.dep: gepetto-viewer-corba.install hpp-gui.checkout
universal_robot.configure.dep: universal_robot.checkout
hpp-universal-robot.configure.dep: example-robot-data.install \
	hpp-universal-robot.checkout
example-robot-data.configure.dep: example-robot-data.checkout
hpp-environments.configure.dep: hpp-environments.checkout
hpp-baxter.configure.dep: hpp-baxter.checkout
hpp_romeo.configure.dep: hpp_romeo.checkout
Universal_Robots_Client_Library.configure.dep: \
	Universal_Robots_Client_Library.checkout
controller_stopper.configure.dep: Universal_Robots_ROS_Driver.checkout
ur_calibration.configure.dep: Universal_Robots_Client_Library.install \
	ur_robot_driver.install Universal_Robots_ROS_Driver.checkout
ur_dashboard_msgs.configure.dep: Universal_Robots_ROS_Driver.checkout
ur_robot_driver.configure.dep: ur_dashboard_msgs.install
sot-universal-robot.configure.dep: sot-dynamic-pinocchio.install \
	dynamic-graph-python.install dynamic_graph_bridge.install \
	dynamic_graph_bridge_msgs.install universal_robot.install \
	sot-universal-robot.checkout
hpp-affordance.configure.dep: hpp-core.install hpp-fcl.install hpp-affordance.checkout
hpp-affordance-corba.configure.dep: hpp-affordance.install hpp-template-corba.install \
 hpp-corbaserver.install hpp-affordance-corba.checkout
anymal-rbprm.configure.dep: anymal-rbprm.checkout
hyq-rbprm.configure.dep: hyq-rbprm.checkout
simple-humanoid-rbprm.configure.dep: simple-humanoid-rbprm.checkout
talos-rbprm.configure.dep: talos-rbprm.checkout
hpp-rbprm.configure.dep: hpp-core.install hpp-centroidal-dynamics.install \
	anymal-rbprm.install hyq-rbprm.install talos-rbprm.install simple-humanoid-rbprm.install \
	hpp-affordance.install ndcurves.install \
	hpp-bezier-com-traj.install hpp-rbprm.checkout
hpp-rbprm-robot-data.configure.dep: hpp-rbprm-robot-data.checkout
hpp-rbprm-corba.configure.dep: hpp-rbprm.install hpp-affordance-corba.install \
 hpp-corbaserver.install hpp-rbprm-robot-data.install hpp-rbprm-corba.checkout
hpp-centroidal-dynamics.configure.dep: eigenpy.install hpp-centroidal-dynamics.checkout
hpp-bezier-com-traj.configure.dep: hpp-centroidal-dynamics.install ndcurves.install hpp-bezier-com-traj.checkout
ndcurves.configure.dep: ndcurves.checkout
eigenpy.configure.dep: eigenpy.checkout
hpp-tools.configure.dep: hpp-tools.checkout
agimus-demos.configure.dep:
agimus-vision.configure.dep: visp_tracker.install
agimus-hpp.configure.dep: dynamic_graph_bridge_msgs.install \
	hpp-manipulation-corba.install agimus_sot_msgs.install \
	hpp-gepetto-viewer.install
agimus-sot.configure.dep: dynamic_graph_bridge.install agimus_sot_msgs.install hpp-manipulation-corba.install
agimus.configure.dep: agimus-hpp.install agimus_sot_msgs.install
agimus_sot_msgs.configure.dep:
agimus-doc.configure.dep: agimus.install agimus-hpp.install agimus-sot.install \
	agimus-demos.install sot-doc.install
gerard-bauzil.configure.dep: gerard-bauzil.checkout
fake-ros.configure.dep: fake-ros.checkout
##### Dynamic graph sot ###################################
dynamic-graph.configure.dep:
dynamic-graph-python.configure.dep: dynamic-graph.install eigenpy.install

dynamic_graph_bridge_msgs.configure.dep:
dynamic_graph_bridge.configure.dep: dynamic-graph.install \
	dynamic_graph_bridge_msgs.install \
	dynamic-graph-python.install \
	sot-dynamic-pinocchio.install \
	sot-core.install
sot-core.configure.dep: dynamic-graph-python.install pinocchio.install
sot-tools.configure.dep: sot-core.install example-robot-data.install
sot-doc.configure.dep: sot-core.install sot-doc.checkout
sot-dynamic-pinocchio.configure.dep: sot-core.install sot-tools.install
roscontrol_sot.configure.dep: dynamic_graph_bridge.install \
	sot-core.install sot-dynamic-pinocchio.install \
	pal_hardware_interfaces.install roscontrol_sot.checkout
pal_hardware_interfaces.configure.dep: pal_msgs.install \
	pal_hardware_interfaces.checkout
pal_hardware_interfaces.configure.dep: pal_hardware_interfaces.checkout
pal_behaviour_msgs.configure.dep: pal_msgs.checkout
pal_common_msgs.configure.dep: pal_msgs.checkout
pal_control_msgs.configure.dep: pal_msgs.checkout
pal_detection_msgs.configure.dep: pal_msgs.checkout
pal_device_msgs.configure.dep: pal_msgs.checkout
pal_interaction_msgs.configure.dep: pal_msgs.checkout
pal_motion_model_msgs.configure.dep: pal_msgs.checkout
pal_multirobot_msgs.configure.dep: pal_msgs.checkout
pal_navigation_msgs.configure.dep: pal_msgs.checkout
pal_simulation_msgs.configure.dep: pal_msgs.checkout
pal_tablet_msgs.configure.dep: pal_msgs.checkout
pal_video_recording_msgs.configure.dep: pal_msgs.checkout
pal_vision_msgs.configure.dep: pal_msgs.checkout
pal_visual_localization_msgs.configure.dep: pal_msgs.checkout
pal_walking_msgs.configure.dep: pal_msgs.checkout
pal_web_msgs.configure.dep: pal_interaction_msgs.install pal_msgs.checkout
pal_wifi_localization_msgs.configure.dep: pal_msgs.checkout
pal_common_msgs.configure.dep: pal_msgs.checkout


##### Vision visp ##########################################
rviz_camera_stream.configure.dep: rviz_camera_stream.checkout
rviz_lighting.configure.dep: rviz_lighting.checkout
librealsense.configure.dep: librealsense.checkout
opencv.configure.dep: opencv.checkout
visp.configure.dep: opencv.install visp.checkout
rgbd_launch.configure.dep:rgbd_launch.checkout
vision_opencv.configure.dep: opencv.install vision_opencv.checkout
visp_bridge.configure.dep: image_pipeline.install visp.install \
	vision_visp.checkout
visp_tracker.configure.dep: visp_bridge.install
visp_auto_tracker.configure.dep: visp_tracker.install
visp_camera_calibration.configure.dep: visp_bridge.install
visp_hand2eye_calibration.configure.dep: visp_bridge.install

image_pipeline.configure.dep: vision_opencv.install image_pipeline.checkout
realsense-ros.configure.dep: realsense-ros.checkout
realsense_gazebo_plugin.configure.dep: realsense_gazebo_plugin.checkout

react_inria.configure.dep: visp_bridge.install react_inria.checkout

# }}}
##################################
# {{{ Targets

status:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".status; \
	done

log:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".log; \
	done

fetch:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".fetch; \
	done

update:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".update; \
	done

%.checkout:
	if [ -d $(@:.checkout=) ]; then \
		echo "$(@:.checkout=) already checkout out."; \
	else \
		git clone ${GIT_QUIET} --recursive -b ${$(@:.checkout=)_branch} ${$(@:.checkout=)_repository}/$(@:.checkout=); \
	fi \

%.fetch:
	if [ "${$(@:.fetch=)_repository}" = "" ]; then \
		echo "$(@:.fetch=) is not referenced"; \
	else \
		cd ${SRC_DIR}/$(@:.fetch=);\
		git fetch ${GIT_QUIET} origin; \
		git fetch ${GIT_QUIET} origin --tags; \
	fi

%.update:
	if [ "${$(@:.update=)_repository}" = "" ]; then \
		echo "$(@:.update=) is not referenced"; \
	else \
		cd ${SRC_DIR}/$(@:.update=);\
		git remote rm origin;\
		git remote add origin ${$(@:.update=)_repository}/$(@:.update=);\
		git fetch origin;\
		git fetch origin --tags;\
		git checkout -q --detach;\
		git branch -f ${$(@:.update=)_branch} origin/${$(@:.update=)_branch};\
		git checkout -q ${$(@:.update=)_branch};\
		git submodule update; \
	fi


%.configure.dep: %.checkout

%.configure: %.configure.dep
	${MAKE} $(@:.configure=).configure_nodep

%.configure_nodep:%.checkout
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_HPP_DIR} -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
			-DENFORCE_MINIMAL_CXX_STANDARD=ON \
			-DINSTALL_DOCUMENTATION=${INSTALL_DOCUMENTATION} \
			-DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-g -O3 -DNDEBUG" \
			${$(@:.configure_nodep=)_extra_flags} ..

%.install:%.configure
	${MAKE} -C ${SRC_DIR}/$(@:.install=)/${BUILD_FOLDER} install

%.install_nodep:%.configure_nodep
	${MAKE} -C ${SRC_DIR}/$(@:.install_nodep=)/${BUILD_FOLDER} install

%.uninstall:
	${MAKE} -C ${SRC_DIR}/$(@:.uninstall=)/${BUILD_FOLDER} uninstall

%.clean:
	${MAKE} -C ${SRC_DIR}/$(@:.clean=)/${BUILD_FOLDER} clean

%.very-clean:
	rm -rf ${SRC_DIR}/$(@:.very-clean=)/${BUILD_FOLDER}/*

%.status:
	@cd ${SRC_DIR}/$(@:.status=); \
	echo \
	"\033[1;36m------- Folder $(@:.status=) ---------------\033[0m"; \
	git --no-pager -c status.showUntrackedFiles=no status --short --branch;\

%.log:
	@cd ${SRC_DIR}/$(@:.log=); \
	if [ -f .git/refs/heads/${$(@:.log=)_branch} ]; then \
		echo -n "$(@:.log=): "; \
		cat .git/refs/heads/${$(@:.log=)_branch}; \
	fi

OpenSceneGraph-3.4.0.checkout:
	if [ -d $(@:.checkout=) ]; then \
		echo "$(@:.checkout=) already checkout out."; \
	else \
		wget http://www.openscenegraph.org/downloads/stable_releases/OpenSceneGraph-3.4.0/source/OpenSceneGraph-3.4.0.zip;\
		cd ${SRC_DIR}; unzip OpenSceneGraph-3.4.0.zip;\
		rm -f OpenSceneGraph-3.4.0.zip;\
	fi

robot_model_py.configure: robot_model_py.configure.dep
	cd ${SRC_DIR}/$(@:.configure=)/xml_reflection;\
	mkdir -p ${BUILD_FOLDER}; \
	cd ${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_HPP_DIR} -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ..
	cd ${SRC_DIR}/$(@:.configure=)/urdf_parser_py;\
	mkdir -p ${BUILD_FOLDER}; \
	cd ${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_HPP_DIR} -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ..

robot_model_py.install: robot_model_py.configure
	${MAKE} -C ${SRC_DIR}/$(@:.install=)/xml_reflection/${BUILD_FOLDER} install; \
	${MAKE} -C ${SRC_DIR}/$(@:.install=)/urdf_parser_py/${BUILD_FOLDER} install;

universal_robot.configure_nodep:
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/ur_description/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/ur_description/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-g -O3 -DNDEBUG" ${$(@:.configure_nodep=)_extra_flags} ..;\
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/ur_gazebo/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/ur_gazebo/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-g -O3 -DNDEBUG" ${$(@:.configure_nodep=)_extra_flags} ..

universal_robot.install_nodep:universal_robot.configure_nodep
	cd ${SRC_DIR}/$(@:.install_nodep=)/ur_description/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install_nodep=)/ur_gazebo/${BUILD_FOLDER};\
	make install

universal_robot.install:universal_robot.configure
	cd ${SRC_DIR}/$(@:.install=)/ur_description/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install=)/ur_gazebo/${BUILD_FOLDER};\
	make install

vision_opencv.configure_nodep:
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/cv_bridge/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/cv_bridge/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/image_geometry/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/image_geometry/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/vision_opencv/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/vision_opencv/${BUILD_FOLDER};

vision_opencv.install_nodep:vision_opencv.configure_nodep
	cd ${SRC_DIR}/$(@:.install_nodep=)/cv_bridge/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install_nodep=)/image_geometry/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install_nodep=)/vision_opencv/${BUILD_FOLDER};

vision_opencv.install:vision_opencv.configure
	cd ${SRC_DIR}/$(@:.install=)/cv_bridge/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install=)/image_geometry/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install=)/vision_opencv/${BUILD_FOLDER};

visp_bridge.configure_nodep:
	mkdir -p ${SRC_DIR}/vision_visp/visp_bridge/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/vision_visp/visp_bridge/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${vision_visp_extra_flags} ..

visp_tracker.configure_nodep:
	mkdir -p ${SRC_DIR}/vision_visp/visp_tracker/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/vision_visp/visp_tracker/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${vision_visp_extra_flags} ..

visp_auto_tracker.configure_nodep:
	mkdir -p ${SRC_DIR}/vision_visp/visp_auto_tracker/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/vision_visp/visp_auto_tracker/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${vision_visp_extra_flags} ..

visp_camera_calibration.configure_nodep:
	mkdir -p ${SRC_DIR}/vision_visp/visp_camera_calibration/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/vision_visp/visp_camera_calibration/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${vision_visp_extra_flags} ..

visp_hand2eye_calibration.configure_nodep:
	mkdir -p ${SRC_DIR}/vision_visp/visp_hand2eye_calibration/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/vision_visp/visp_hand2eye_calibration/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${vision_visp_extra_flags} ..

visp_bridge.install_nodep:
	cd ${SRC_DIR}/vision_visp/visp_bridge/${BUILD_FOLDER}; \
	make install

visp_tracker.install_nodep:
	cd ${SRC_DIR}/vision_visp/visp_tracker/${BUILD_FOLDER}; \
	make install

visp_auto_tracker.install_nodep:
	cd ${SRC_DIR}/vision_visp/visp_auto_tracker/${BUILD_FOLDER}; \
	make install

visp_camera_calibration.install_nodep:
	cd ${SRC_DIR}/vision_visp/visp_camera_calibration/${BUILD_FOLDER}; \
	make install

visp_hand2eye_calibration.install_nodep:
	cd ${SRC_DIR}/vision_visp/visp_hand2eye_calibration/${BUILD_FOLDER}; \
	make install

visp_bridge.install: visp_bridge.configure
	cd ${SRC_DIR}/vision_visp/visp_bridge/${BUILD_FOLDER}; \
	make install

visp_tracker.install: visp_tracker.configure
	cd ${SRC_DIR}/vision_visp/visp_tracker/${BUILD_FOLDER}; \
	make install
visp_auto_tracker.install: visp_auto_tracker.configure
	cd ${SRC_DIR}/vision_visp/visp_auto_tracker/${BUILD_FOLDER}; \
	make install

visp_camera_calibration.install:visp_camera_calibration.configure
	cd ${SRC_DIR}/vision_visp/visp_camera_calibration/${BUILD_FOLDER}; \
	make install

visp_hand2eye_calibration.install: visp_hand2eye_calibration.configure
	cd ${SRC_DIR}/vision_visp/visp_hand2eye_calibration/${BUILD_FOLDER}; \
	make install

image_pipeline.configure_nodep:
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/camera_calibration/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/camera_calibration/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/depth_image_proc/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/depth_image_proc/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/image_pipeline/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/image_pipeline/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/image_proc/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/image_proc/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	cd ${SRC_DIR}/$(@:.configure_nodep=)/image_publisher/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	cd ${SRC_DIR}/$(@:.configure_nodep=)/image_rotate/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	cd ${SRC_DIR}/$(@:.configure_nodep=)/stereo_image_proc/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..

image_pipeline.install_nodep:image_pipeline.configure_nodep
	cd ${SRC_DIR}/$(@:.install_nodep=)/camera_calibration/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install_nodep=)/depth_image_proc/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install_nodep=)/image_pipeline/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install_nodep=)/image_proc/${BUILD_FOLDER};\
	make install;\
	cd ${SRC_DIR}/$(@:.install_nodep=)/image_publisher/${BUILD_FOLDER};\
	make install;\
	cd ${SRC_DIR}/$(@:.install_nodep=)/image_rotate/${BUILD_FOLDER};\
	make install;\
	cd ${SRC_DIR}/$(@:.install_nodep=)/stereo_image_proc/${BUILD_FOLDER};\
	make install

image_pipeline.install:vision_opencv.install image_pipeline.configure
	cd ${SRC_DIR}/$(@:.install=)/camera_calibration/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install=)/depth_image_proc/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install=)/image_pipeline/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install=)/image_proc/${BUILD_FOLDER};\
	make install;\
	cd ${SRC_DIR}/$(@:.install=)/image_publisher/${BUILD_FOLDER};\
	make install;\
	cd ${SRC_DIR}/$(@:.install=)/image_rotate/${BUILD_FOLDER};\
	make install;\
	cd ${SRC_DIR}/$(@:.install=)/stereo_image_proc/${BUILD_FOLDER};\
	make install

realsense-ros.checkout:
	if [ -d $(@:.checkout=) ]; then \
		echo "$(@:.checkout=) already checkout out."; \
	else \
		git clone ${GIT_QUIET} --recursive -b ${$(@:.checkout=)_branch} ${$(@:.checkout=)_repository}/realsense realsense-ros; \
	fi \

realsense-ros.configure_nodep:
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/realsense2_camera/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/realsense2_camera/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/realsense2_description/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/realsense2_description/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\

realsense-ros.install_nodep:realsense-ros.configure_nodep
	cd ${SRC_DIR}/$(@:.install_nodep=)/realsense2_camera/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install_nodep=)/realsense2_description/${BUILD_FOLDER};\
	make install; \

realsense-ros.install:librealsense.install realsense-ros.configure
	cd ${SRC_DIR}/$(@:.install=)/realsense2_camera/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install=)/realsense2_description/${BUILD_FOLDER};\
	make install; \

pal_behaviour_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_common_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_control_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_detection_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_device_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_interaction_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_motion_model_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_multirobot_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_navigation_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_simulation_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_tablet_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_video_recording_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_vision_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_visual_localization_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_walking_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_web_msgs.configure_nodep: pal_interaction_msgs.install
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_wifi_localization_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/pal_msgs/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${pal_msgs_extra_flags} ..

pal_behaviour_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_common_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_control_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_detection_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_device_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_interaction_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_motion_model_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_multirobot_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_navigation_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_simulation_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_tablet_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_video_recording_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_vision_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_visual_localization_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_walking_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_web_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_wifi_localization_msgs.install_nodep:
	cd ${SRC_DIR}/pal_msgs/${@:.install_nodep=}/${BUILD_FOLDER}; \
	make install

pal_behaviour_msgs.install: pal_behaviour_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_common_msgs.install: pal_common_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_control_msgs.install: pal_control_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_detection_msgs.install: pal_detection_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_device_msgs.install: pal_device_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_interaction_msgs.install: pal_interaction_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_motion_model_msgs.install: pal_motion_model_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_multirobot_msgs.install: pal_multirobot_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_navigation_msgs.install: pal_navigation_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_simulation_msgs.install: pal_simulation_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_tablet_msgs.install: pal_tablet_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_video_recording_msgs.install: pal_video_recording_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_vision_msgs.install: pal_vision_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_visual_localization_msgs.install: pal_visual_localization_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_walking_msgs.install: pal_walking_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_web_msgs.install: pal_interaction_msgs.install pal_web_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_wifi_localization_msgs.install: pal_wifi_localization_msgs.configure
	cd ${SRC_DIR}/pal_msgs/${@:.install=}/${BUILD_FOLDER}; \
	make install

pal_msgs.install: pal_wifi_localization_msgs.install \
	pal_web_msgs.install pal_walking_msgs.install \
	pal_visual_localization_msgs.install pal_vision_msgs.install \
	pal_video_recording_msgs.install pal_tablet_msgs.install \
	pal_simulation_msgs.install pal_navigation_msgs.install \
	pal_multirobot_msgs.install pal_motion_model_msgs.install \
	pal_interaction_msgs.install pal_hardware_interfaces.install \
	pal_device_msgs.install pal_detection_msgs.install \
	pal_control_msgs.install pal_common_msgs.install \
	pal_behaviour_msgs.install 

controller_stopper.configure_nodep:
	mkdir -p ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${Universal_Robots_ROS_Driver_extra_flags} ..

ur_calibration.configure_nodep:
	mkdir -p ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${Universal_Robots_ROS_Driver_extra_flags} ..

ur_dashboard_msgs.configure_nodep:
	mkdir -p ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${Universal_Robots_ROS_Driver_extra_flags} ..

ur_robot_driver.configure_nodep:
	mkdir -p ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${Universal_Robots_ROS_Driver_extra_flags} ..

controller_stopper.install_nodep:
	cd ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.install_nodep=)/${BUILD_FOLDER}; \
	make install

ur_calibration.install_nodep:
	cd ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.install_nodep=)/${BUILD_FOLDER}; \
	make install

ur_dashboard_msgs.install_nodep:
	cd ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.install_nodep=)/${BUILD_FOLDER}; \
	make install

ur_robot_driver.install_nodep:
	cd ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.install_nodep=)/${BUILD_FOLDER}; \
	make install

controller_stopper.install: controller_stopper.configure
	cd ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.install=)/${BUILD_FOLDER}; \
	make install

ur_calibration.install: ur_robot_driver.install \
	Universal_Robots_Client_Library.install ur_calibration.configure
	cd ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.install=)/${BUILD_FOLDER}; \
	make install

ur_dashboard_msgs.install: ur_dashboard_msgs.configure
	cd ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.install=)/${BUILD_FOLDER}; \
	make install

ur_robot_driver.install: Universal_Robots_Client_Library.install \
	ur_dashboard_msgs.install ur_robot_driver.configure
	cd ${SRC_DIR}/Universal_Robots_ROS_Driver/$(@:.install=)/${BUILD_FOLDER}; \
	make install

Universal_Robots_ROS_Driver.install: controller_stopper.install \
	ur_calibration.install ur_dashboard_msgs.install ur_robot_driver.install

react_inria.checkout:
	if [ -d react_inria ]; then \
		echo "react_inria already checkout out."; \
	else \
		git clone ${GIT_QUIET} --recursive -b ${react_inria_branch} https://oauth2:glpat-LUsL1Gx9_xMFqWALBvP_@gitlab.laas.fr/rob4fam/react_inria.git; \
	fi \


ros_control.configure_nodep:
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/controller_manager_msgs/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/controller_manager_msgs/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/hardware_interface/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/hardware_interface/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/combined_robot_hw/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/combined_robot_hw/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/controller_interface/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/controller_interface/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/joint_limits_interface/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/joint_limits_interface/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/transmission_interface/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/transmission_interface/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/controller_manager/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/controller_manager/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ${$(@:.configure_nodep=)_extra_flags} ..;\

ros_control.install_nodep:ros_control.configure_nodep
	cd ${SRC_DIR}/$(@:.install_nodep=)/controller_manager_msgs/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install_nodep=)/hardware_interface/${BUILD_FOLDER};\
	make install;\
	cd ${SRC_DIR}/$(@:.install_nodep=)/combined_robot_hw/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install_nodep=)/controller_interface/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install_nodep=)/joint_limits_interface/${BUILD_FOLDER};\
	make install;\
	cd ${SRC_DIR}/$(@:.install_nodep=)/transmission_interface/${BUILD_FOLDER};\
	make install;\
	cd ${SRC_DIR}/$(@:.install_nodep=)/controller_manager/${BUILD_FOLDER};\
	make install;\


ros_control.install:ros_control.configure
	cd ${SRC_DIR}/$(@:.install=)/controller_manager_msgs/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install=)/hardware_interface/${BUILD_FOLDER};\
	make install;\
	cd ${SRC_DIR}/$(@:.install=)/combined_robot_hw/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install=)/controller_interface/${BUILD_FOLDER};\
	make install; \
	cd ${SRC_DIR}/$(@:.install=)/joint_limits_interface/${BUILD_FOLDER};\
	make install;\
	cd ${SRC_DIR}/$(@:.install=)/transmission_interface/${BUILD_FOLDER};\
	make install;\
	cd ${SRC_DIR}/$(@:.install=)/controller_manager/${BUILD_FOLDER};\
	make install;\

# }}}

# vim: foldmethod=marker foldlevel=0
