# Create an image to run agimus in a Docker.

You may run `build_image.sh` to build the docker image.

## Details on the image build

The image is built from memmos.laas.fr:5000/gepetto/buildfarm/ros:18.04. I think it can be built
from osrf/ros:melodic-desktop-bionic, which is publicly available.

The git repo rob4fam-models is private. It is must be first cloned in the host
and then copied into the docker so that building the image does not require user
login and password.

A few patches are applied in order to make the sources catkin compliant. They affect
the dependency (pinocchio depends on hpp-fcl) and modify some default CMake options.

## Network configuration

Make sure you run the image with option `--net=host`. Currently,
- `hppcorbaserver` should be started within the docker.
- `gepetto-gui` should be started outside the docker.
