FROM ubuntu:20.04
RUN apt update -y && apt install ca-certificates -y && \
    sh -c 'echo "deb https://mirrors.ustc.edu.cn/ubuntu/ focal main restricted universe multiverse\ndeb https://mirrors.ustc.edu.cn/ubuntu/ focal-updates main restricted universe multiverse\ndeb https://mirrors.ustc.edu.cn/ubuntu/ focal-backports main restricted universe multiverse\ndeb https://mirrors.ustc.edu.cn/ubuntu/ focal-security main restricted universe multiverse" > /etc/apt/sources.list' && \
    apt update -y && DEBIAN_FRONTEND=noninteractive apt install -y tzdata && \
    apt install -y dirmngr gnupg2 && \
    sh -c 'echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ focal main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 &&\
    rm -rf /var/lib/apt/lists/*

# 一键换源
RUN apt update -y \
    && apt install -y wget python3 python3-yaml python3-distro\
    # rosdep 更新
    && echo "chooses:\n" > fish_install.yaml \
    && echo "- {choose: 3, desc: '一键配置:rosdep(小鱼的rosdepc,又快又好用)'}\n" >> fish_install.yaml \
    && wget http://fishros.com/install  -O fishros && /bin/bash fishros \
    # 进行最后的清理
    && rm -rf fish_install.yaml \
    && rm -rf /var/lib/apt/lists/*  /tmp/* /var/tmp/* \
    && apt clean && apt autoclean 

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8


RUN apt-get update -y && \
    apt-get install -y ros-noetic-ros-base python3-rosdep python3-rosinstall \
    python3-rosinstall-generator python3-wstool python3-catkin-tools build-essential && \
    pip install scipy -i https://pypi.tuna.tsinghua.edu.cn/simple && \
    pip install casadi==3.5.5 -i https://pypi.tuna.tsinghua.edu.cn/simple && \
    pip install pinocchio==0.4.3 -i https://pypi.tuna.tsinghua.edu.cn/simple


RUN apt-get update -y && \
    apt-get install -y ros-noetic-gazebo-ros ros-noetic-gazebo-ros-control \
    ros-noetic-controller-manager ros-noetic-joint-state-controller \
    ros-noetic-joint-state-publisher ros-noetic-ros-control ros-noetic-ros-controllers &&\
    rm -rf /var/lib/apt/lists/* 

# COPY ./ros_entrypoint.sh /

# ENTRYPOINT ["bash /ros_entrypoint.sh"]
# CMD ["bash"]

ENV CATKIN_WS=/root/catkin_ws

COPY ./ $CATKIN_WS/src/
ENV ROS_DISTRO=noetic
RUN apt-get update -y && \
    cd ${CATKIN_WS} && rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    rm -rf /var/lib/apt/lists/* 
