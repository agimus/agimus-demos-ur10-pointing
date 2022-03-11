FROM osrf/ros:melodic-desktop-bionic

ENV CMAKE_PREFIX_PATH=/opt/openrobots

RUN echo deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub bionic robotpkg > /etc/apt/sources.list.d/robotpkg.list
RUN echo deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub bionic robotpkg >> /etc/apt/sources.list.d/robotpkg.list

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | \
    apt-key add -

# Configure catkin workspace
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -qy python3-vcstool python-catkin-tools git cmake vim g++ ros-melodic-catkin ccache \
      ros-melodic-amcl ros-melodic-backward-ros ros-melodic-catkin ros-melodic-cmake-modules ros-melodic-ddynamic-reconfigure-python \
      ros-melodic-depthimage-to-laserscan ros-melodic-driver-base ros-melodic-eband-local-planner ros-melodic-fake-localization \
      ros-melodic-global-planner ros-melodic-image-transport ros-melodic-image-transport-plugins ros-melodic-image-view ros-melodic-image-view2 \
      ros-melodic-joy ros-melodic-laser-filters ros-melodic-libuvc-camera ros-melodic-move-base ros-melodic-moveit-commander \
      ros-melodic-moveit-planners-ompl ros-melodic-moveit-ros-move-group ros-melodic-moveit-ros-planning-interface \
      ros-melodic-moveit-ros-visualization ros-melodic-moveit-simple-controller-manager ros-melodic-navigation-layers \
      ros-melodic-octomap-server ros-melodic-openni-launch ros-melodic-openni2-camera ros-melodic-openni2-launch \
      ros-melodic-pointcloud-to-laserscan ros-melodic-pr2-description ros-melodic-qt-gui-py-common ros-melodic-rgbd-launch \
      ros-melodic-robot-state-publisher ros-melodic-rosbash ros-melodic-roslaunch ros-melodic-roslint ros-melodic-rqt-tf-tree \
      ros-melodic-rviz ros-melodic-sick-tim ros-melodic-slam-gmapping ros-melodic-slam-toolbox ros-melodic-smach-ros \
      ros-melodic-spatio-temporal-voxel-layer ros-melodic-teb-local-planner ros-melodic-tf2-geometry-msgs ros-melodic-twist-mux \
      ros-melodic-urdf-test ros-melodic-uvc-camera ros-melodic-video-stream-opencv ros-melodic-video-stream-opencv-dbgsym ros-melodic-visp-bridge \
      ros-melodic-tf-conversions python-omniorb libboost-dev liburdfdom-dev libassimp-dev libeigen3-dev libgraphviz-dev ros-melodic-xacro \
      libccd-dev ros-melodic-octomap ros-melodic-resource-retriever ros-melodic-srdfdom libltdl-dev python-omniorb python-matplotlib python-scipy libxml2 \
      libtinyxml2-dev libpcre3-dev libcdd-dev libomniorb4-dev omniidl omniidl-python gazebo9 libgazebo9-dev ros-melodic-polled-camera \
      ros-melodic-four-wheel-steering-msgs ros-melodic-urdf-geometry-parser python-tqdm \
      robotpkg-py27-qt5-gepetto-viewer-corba robotpkg-py27-qt5-hpp-gui \
      robotpkg-py27-qt5-hpp-plot robotpkg-pyrene-motions \
      robotpkg-talos-simulation oxygen-icon-theme \
      mesa-utils net-tools novnc terminator firefox \
      x11-apps tigervnc-standalone-server tigervnc-xorg-extension xfce4 \
      ipython emacs pybind11-dev  doxygen openssh-client

RUN git config --global url."https://github.com/".insteadOf git://github.com/
RUN mkdir -p ~/catkin_ws/src
WORKDIR /root/catkin_ws
RUN catkin init && catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_BUILD_TESTING=0 -DBUILD_TESTING=OFF -DINSTALL_DOCUMENTATION=ON

RUN mkdir ~/.vnc \
 && echo -e "#!/bin/sh\nstartxfce4" > ~/.vnc/xstartup \
 && chmod +x ~/.vnc/xstartup

COPY passwd /root/.vnc/passwd
COPY launch-gepetto-gui.sh /root/catkin_ws/install/bin/gepetto-gui

COPY ./pal_tiago.repos src/pal_tiago.repos
RUN cd src && vcs-import --recursive --input pal_tiago.repos

RUN . /opt/ros/melodic/setup.sh && catkin build

COPY ./gepetto_tiago.repos src/gepetto_tiago.repos
RUN cd src && vcs-import --recursive --input gepetto_tiago.repos

COPY ./patches src/patches
COPY ./patches/apply.sh src/apply_patches.sh
RUN cd src/ && bash apply_patches.sh

RUN . install/setup.sh && catkin build -j 1

# Clone manually rob4fam-models
COPY rob4fam-models src/rob4fam-models
RUN git -C src/rob4fam-models remote set-url origin https://gitlab.laas.fr/rob4fam/rob4fam-models.git
RUN catkin build rob4fam-models

COPY config.sh /root/config.sh
RUN echo "source ~/config.sh" >> ~/.bashrc

# vim: ft=dockerfile
