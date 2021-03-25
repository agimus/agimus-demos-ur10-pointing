FROM memmos.laas.fr:5000/jmirabel/agimus-demos/tiago_deburring:4_vnc

RUN apt install -q -y iproute2
COPY set_env_for_tiago /root/set_env_for_tiago
RUN echo -e "source ~/set_env_for_tiago" >> ~/.bashrc
RUN mkdir -p /root/.ssh && echo -e "Host tiago-wired\n  Hostname 10.68.0.1\n  User pal\n" >> /root/.ssh/config


# vim: ft=dockerfile
