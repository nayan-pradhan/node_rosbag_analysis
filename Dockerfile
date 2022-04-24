FROM ros:noetic

LABEL maintainer="nayan.pradhan@hotmail.com"

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
        vim \
        git \
        iputils-ping \
        net-tools \
        netcat \
        ssh \
        rsync \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -q -y --no-install-recommends \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool build-essential \
        python3-rosdep \
        python3-pip \
        python3-distutils \
        python3-psutil \
        && rm -rf /var/lib/apt/lists/*

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get upgrade -q -y \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/$user/${init_workspace}/src
RUN mkdir -p /home/$user/${init_workspace}/data

RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; catkin_init_workspace /home/$user/${init_workspace}/src"
RUN /bin/bash -c ". /opt/ros/noetic/setup.bash; cd /home/$user/${init_workspace}; catkin_make"
RUN /bin/bash -c "echo source /opt/ros/noetic/setup.bash >> /home/$user/.bashrc"
RUN /bin/bash -c "echo source /home/$user/${init_workspace}/devel/setup.bash >> /home/$user/.bashrc"
RUN /bin/bash -c "chown -R $user:$user /home/$user/"

