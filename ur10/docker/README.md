# Create an image to run agimus in a Docker.

You may run `build_image.sh` to build the docker image.

## Images

Tags follow the rules. They start with the *<version_number>*.
Then, suffix *_vnc* is added to the version with VNC server configured.
Then, suffix *_robot* is added to the version configured for being connected to Tiago robot.

The *_vnc* tagged images have the vnc server command as entrypoint. The VNC password is *hpphpp*.
To connect to the server, you may run:

```
xtigervncviewer -SecurityTypes VncAuth,TLSVnc :3
```

## Dockerfiles

The docker files should be run in the following order:
- Dockerfile, agimus installation, without gepetto-viewer.
- Dockerfile.vnc, install a VNC server and gepetto-gui.
- Dockerfile.robot, adds a bash script that sets default environment variables.

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
