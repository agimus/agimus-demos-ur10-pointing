FROM gitlab.laas.fr:4567/rob4fam/docker/tiago_deburring:2_vnc

RUN apt update -qy && DEBIAN_FRONTEND=noninteractive apt install -qy iproute2
COPY set_env_for_tiago /root/set_env_for_tiago
RUN echo "source ~/set_env_for_tiago" >> ~/.bashrc
RUN mkdir -p /root/.ssh && echo -e "Host tiago-wired\n  Hostname 10.68.0.1\n  User pal\n" >> /root/.ssh/config


# vim: ft=dockerfile
