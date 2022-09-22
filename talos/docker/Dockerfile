FROM registry.gitlab.com/pal-robotics-public/talos-simulation:latest

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

RUN echo deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub bionic robotpkg > /etc/apt/sources.list.d/robotpkg.list
RUN echo deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub bionic robotpkg >> /etc/apt/sources.list.d/robotpkg.list

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | \
    apt-key add -

# Configure catkin workspace
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -qy python3-vcstool python-catkin-tools git cmake vim g++ ros-melodic-catkin ccache \
    python-omniorb libboost-dev liburdfdom-dev libassimp-dev libeigen3-dev libgraphviz-dev ros-melodic-xacro \
      libccd-dev ros-melodic-octomap ros-melodic-resource-retriever ros-melodic-srdfdom libltdl-dev python-omniorb python-matplotlib python-scipy libxml2 \
      libtinyxml2-dev libpcre3-dev libcdd-dev libomniorb4-dev omniidl omniidl-python python-tqdm \
      robotpkg-qt5-osgqt robotpkg-openscenegraph ffmpeg libpythonqt-qt5-common-dev libpythonqt-qtall-qt5-common-dev libpythonqt-qt5-python2-dev \
      libpythonqt-qtall-qt5-python2-dev qtbase5-dev libtinyxml-dev mesa-common-dev libglvnd-dev libgl1-mesa-dev omniorb-nameserver omniorb-idl omniidl-python \
      robotpkg-qt5-qgv oxygen-icon-theme robotpkg-pal-msgs robotpkg-pal-hardware-interfaces robotpkg-pyrene-motions robotpkg-py27-eigenpy \
      mesa-utils net-tools novnc terminator firefox \
      x11-apps tigervnc-standalone-server tigervnc-xorg-extension xfce4 \
      ipython emacs pybind11-dev  doxygen openssh-client gdb gnuplot \
      ros-melodic-visp-bridge

RUN mkdir ~/.vnc \
 && echo -e "#!/bin/sh\nstartxfce4" > ~/.vnc/xstartup \
 && chmod +x ~/.vnc/xstartup

WORKDIR /root/catkin_ws

RUN mkdir -p ~/catkin_ws/src

COPY passwd /root/.vnc/passwd
COPY config.sh /root/config.sh
COPY setup_build.sh /root/catkin_ws/setup_build.sh

RUN git config --global url."https://github.com/".insteadOf git://github.com/
RUN . ./setup_build.sh && catkin init && catkin config --install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCATKIN_BUILD_TESTING=0 -DBUILD_TESTING=OFF -DINSTALL_DOCUMENTATION=OFF

COPY ./gepetto_talos.repos src/gepetto_talos.repos
COPY ./patches src/patches
COPY ./patches/apply.sh src/apply_patches.sh

RUN cd src && vcs-import --recursive --input gepetto_talos.repos
RUN cd src/ && bash apply_patches.sh

RUN . /root/catkin_ws/setup_build.sh && cd src && catkin build -j 1

# # Clone manually rob4fam-models
# COPY rob4fam-models src/rob4fam-models
# RUN git -C src/rob4fam-models remote set-url origin https://gitlab.laas.fr/rob4fam/rob4fam-models.git
# RUN catkin build rob4fam-models

# # vim: ft=dockerfile
