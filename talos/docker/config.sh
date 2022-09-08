source ~/catkin_ws/install/setup.bash
# Add /opt/openrobots in environment variables
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/openrobots/lib
export PYTHONPATH=$PYTHONPATH:/opt/openrobots/lib/python2.7/site-packages
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/opt/openrobots/share
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/openrobots

export CC=/usr/lib/ccache/gcc
export CXX=/usr/lib/ccache/g++
