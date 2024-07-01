FROM osrf/ros:noetic-desktop-full-focal

# Instalação de pacotes necessários
RUN apt-get update && apt-get install -y \
    nano \
    git \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get install -y --reinstall ca-certificates \
    && apt-get update \
    && apt-get install -y software-properties-common \
    && apt-get install -y sudo \
    && apt-get update \
    && apt install build-essential python3-rosdep python3-catkin-tools -y \
    && apt install libusb-dev python3-osrf-pycommon libspnav-dev libbluetooth-dev libcwiid-dev libgoogle-glog-dev -y \
    && apt install ros-noetic-mavros ros-noetic-octomap-ros -y \
    && apt-get update
    
RUN sudo apt-get remove --purge ibus
    
# Add user
RUN useradd -rm -d /home/ubuntu -p $(perl -e 'print crypt($ARGV[0], "password")' '') -s /bin/bash -g root -G sudo -u 1001 ubuntu

USER root

RUN mkdir -p /home/ubuntu/bebop_ws/src

RUN cd /home/ubuntu/bebop_ws/src \
 && git clone https://github.com/antonellabarisic/parrot_arsdk.git \
 && git clone https://github.com/ros-teleop/teleop_twist_keyboard \
 && git clone https://github.com/ethz-asl/mav_comm \
 && git clone https://github.com/ViktorCVS/Bebop_simulation.git \
 && git clone https://github.com/ViktorCVS/Bebop_control.git \
 && git clone https://github.com/ros-drivers/joystick_drivers

RUN cd /home/ubuntu/bebop_ws/src/parrot_arsdk \
 && git config --global --add safe.directory /home/ubuntu/bebop_ws/src/parrot_arsdk \
 && git checkout noetic_dev

RUN cd /home/ubuntu/bebop_ws \
 && apt-get update \
 && apt-get install libavahi-client-dev -y \
 && ln -s /usr/bin/python3 /usr/bin/python

WORKDIR /home/ubuntu/bebop_ws

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make > build_log_1.txt 2>&1 || tail -n 100 build_log_1.txt"

RUN cd /home/ubuntu/bebop_ws/src \
 && git clone https://github.com/ViktorCVS/bebop_autonomy.git

RUN cd /home/ubuntu/bebop_ws && rosdep update && rosdep install -y --from-paths . --ignore-src

RUN apt install ros-noetic-joy ros-noetic-joy-teleop ros-noetic-teleop-twist-joy -y

RUN echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ubuntu/bebop_ws/devel/lib/parrot_arsdk" >> /home/ubuntu/.bashrc

RUN /bin/bash -c "cd /home/ubuntu/bebop_ws && source /opt/ros/noetic/setup.bash && catkin_make > build_log_2.txt 2>&1 || tail -n 100 build_log_2.txt"

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "cd /home/ubuntu/bebop_ws && source devel/setup.bash" >> /root/.bashrc

RUN echo "Imagem criada com sucesso."
