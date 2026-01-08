FROM nvidia/cuda:12.8.0-devel-ubuntu24.04

ARG RELEASE
ARG LAUNCHPAD_BUILD_ARCH
ENV USER=user1
ARG PASSWORD=user1
SHELL ["/bin/bash", "-c"]
RUN ln -fs /usr/share/zoneinfo/Europe/Rome /etc/localtime && \
    apt update &&\
    apt install --no-install-recommends -y -qq sudo git cmake && \
    rm -rf /var/lib/apt/lists/*

RUN addgroup ${USER} \
    && useradd -ms /bin/bash ${USER} -g ${USER} \
    && echo "${USER}:${PASSWORD}" | chpasswd \
    && usermod -a -G sudo ${USER} \
    && sed -i.bak -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

USER ${USER}
WORKDIR /home/${USER}

RUN sudo apt update && DEBIAN_FRONTEND=noninteractive sudo apt install ffmpeg libsm6 libxext6 -y

# ROS2 Install
ENV ROS_DISTRO=jazzy
RUN sudo apt update && sudo apt install -y locales && sudo locale-gen en_US en_US.UTF-8 \
    && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8
    
RUN sudo apt install software-properties-common -y\
    && sudo add-apt-repository universe \
    && sudo apt update && sudo apt install curl -y

RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    sudo apt install /tmp/ros2-apt-source.deb

RUN sudo apt update && sudo apt install -y ros-$ROS_DISTRO-ros-base \
    ros-$ROS_DISTRO-sensor-msgs ros-$ROS_DISTRO-geometry-msgs \
    ros-$ROS_DISTRO-action-msgs ros-$ROS_DISTRO-actionlib-msgs \
    ros-$ROS_DISTRO-nav2-rviz-plugins ros-$ROS_DISTRO-rviz2 ros-dev-tools \
    ros-$ROS_DISTRO-nav2-msgs ros-$ROS_DISTRO-nav2-simple-commander  \
    ros-$ROS_DISTRO-nav2-costmap-2d

RUN sudo apt update && sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp -y
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USER}/.bashrc

# VS code
RUN wget -O- https://packages.microsoft.com/keys/microsoft.asc | sudo gpg --dearmor | sudo tee /usr/share/keyrings/vscode.gpg && \
    echo deb [arch=amd64 signed-by=/usr/share/keyrings/vscode.gpg] https://packages.microsoft.com/repos/vscode stable main | sudo tee /etc/apt/sources.list.d/vscode.list
RUN sudo apt update && sudo apt install -y code
# Install VisualStudio Code extensions
RUN code --install-extension ms-vscode.cpptools \
		--install-extension ms-vscode.cpptools-themes \
		--install-extension ms-vscode.cmake-tools \
                --install-extension ms-python.python \
                --install-extension eamodio.gitlens

# Git Setup
ARG GIT_USERNAME
ARG GIT_USER_EMAIL
RUN git config --global user.name ${GIT_USERNAME} && git config --global user.email ${GIT_USER_EMAIL}
ARG GIT_TOKEN
RUN sudo apt update && sudo apt install -y python3-pip

# Cuda ENV
ENV TORCH_CUDA_ARCH_LIST="8.6"
ENV FORCE_CUDA="1"

# Grounding DINO
RUN pip3 install torch torchvision transformers addict yapf timm numpy opencv-python supervision>=0.22.0 pycocotools --break-system-packages
RUN git clone https://github.com/SimoneMic/GroundingDINO.git 
RUN cd GroundingDINO && git switch cuda12.8-fix && pip3 install --break-system-packages -e .
RUN cd GroundingDINO && mkdir weights && cd weights && wget -q https://github.com/IDEA-Research/GroundingDINO/releases/download/v0.1.0-alpha/groundingdino_swint_ogc.pth
# TODO make script that downloads cache

# Ultralytics
RUN pip3 install ultralytics open3d shapely --break-system-packages && pip3 install --no-cache-dir "git+https://github.com/ultralytics/CLIP.git" --break-system-packages

# Detection Pipeline
RUN sudo apt update && sudo apt install -y ros-$ROS_DISTRO-pcl-ros
RUN mkdir -p ws_ros2/src && cd ws_ros2/src && git clone https://${GIT_TOKEN}@github.com/hsp-iit/manipulation-approach &&\
    cd .. && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install
RUN echo "source /home/${USER}/ws_ros2/install/local_setup.bash" >> /home/${USER}/.bashrc


# Cleanup
RUN sudo apt update && sudo apt install -y iproute2 unzip terminator bash-completion gedit locate && sudo rm -rf /var/lib/apt/lists/* && sudo updatedb
