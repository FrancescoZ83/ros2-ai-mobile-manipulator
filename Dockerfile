FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# 1. System utilities, C++17 dev tools, core dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    gdb \
    git \
    curl \
    gnupg \
    lsb-release \
    sudo \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pykdl \
    libeigen3-dev \
    libopencv-dev \
    liborocos-kdl-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. Gazebo Ignition Fortress (official OSRF repository)
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends \
    ignition-fortress \
    && rm -rf /var/lib/apt/lists/*

# 3. ROS 2 stacks: Navigation, MoveIt 2, BehaviorTree, Kinematics, Control, Vision
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Navigation & SLAM
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-pointcloud-to-laserscan \
    # Manipulation & Orchestration
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-behaviortree-cpp-v3 \
    # Kinematics
    ros-humble-kdl-parser \
    # ros2_control & Ignition Gazebo integration
    ros-humble-ros-ign-bridge \
    ros-humble-ros-gz \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-ign-ros2-control \
    ros-humble-ign-ros2-control-demos \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    # Vision & sensor pipelines
    ros-humble-cv-bridge \
    ros-humble-vision-opencv \
    ros-humble-vision-msgs \
    ros-humble-image-pipeline \
    ros-humble-usb-cam \
    ros-humble-tf-transformations \
    ros-humble-turtlesim \
    # DDS more stable with Nav2
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# 4. Python ML / inference stack (CUDA 12.1 wheels)
RUN pip3 install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cu121 && \
    pip3 install --no-cache-dir "numpy<2" scipy transforms3d onnxruntime-gpu "setuptools==58.2.0"

# 5. Non-root user matching host UID/GID
ARG USER_ID=1000
ARG GROUP_ID=1000
RUN groupadd --gid $GROUP_ID user && \
    useradd --uid $USER_ID --gid $GROUP_ID --create-home --shell /bin/bash user && \
    echo "user:user" | chpasswd && \
    echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 6. Environment
USER user
ENV HOME=/home/user
ENV DISPLAY=:0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all


RUN rosdep update && mkdir -p ${HOME}/ros2_ws/src

# 7. Shell sourcing
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME}/.bashrc && \
    echo "if [ -f ${HOME}/ros2_ws/install/setup.bash ]; then source ${HOME}/ros2_ws/install/setup.bash; fi" >> ${HOME}/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ${HOME}/.bashrc && \
    echo "export _colcon_cd_root=/opt/ros/${ROS_DISTRO}/" >> ${HOME}/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ${HOME}/.bashrc

WORKDIR ${HOME}/ros2_ws
CMD ["/bin/bash"]
