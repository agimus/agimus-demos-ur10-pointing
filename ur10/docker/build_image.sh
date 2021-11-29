#!/bin/bash

ros_distro=noetic
address=gitlab.laas.fr:4567
name=rob4fam/docker/ur10-pointing
tag=4

if [ ! -d "rob4fam-models" ]; then
  git clone --recursive https://gitlab.laas.fr/rob4fam/rob4fam-models.git
else
  echo "rob4fam-models already checked out."
fi
docker build --tag $address/$name:$tag -f Dockerfile.$ros_distro .
