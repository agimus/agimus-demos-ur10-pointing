FROM memmos.laas.fr:5000/jmirabel/agimus-demos/tiago_deburring:4_vnc

RUN apt install -q -y iproute2
COPY set_env_for_tiago /root/set_env_for_tiago
RUN echo -e "source ~/set_env_for_tiago" >> ~/.bashrc

# vim: ft=dockerfile
