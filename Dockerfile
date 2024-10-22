# Use ROS 2 Humble base image
FROM osrf/ros:humble-desktop

RUN apt-get update && apt-get install -y \
        wget \
        iproute2 \
        python3-colcon-common-extensions \
        python3-pip \
        ros-humble-plotjuggler-ros \
        && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

RUN wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add -

RUN apt-get update && apt-get install -y \
        libgz-transport13-dev \
        && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install setuptools==58.2.0 numpy==1.21.5 control cvxopt

RUN mkdir /ros_workspace

COPY ./px4_msgs /ros_workspace/src/px4_msgs
COPY ./px4_ssr /ros_workspace/src/px4_ssr
COPY ./px4_offboard_control /ros_workspace/src/px4_offboard_control

# Set the workspace as the working directory
WORKDIR /ros_workspace

# Build your ROS workspace
RUN /bin/bash -c '. /opt/ros/humble/setup.bash; colcon build --symlink-install'

# Source the ROS 2 and workspace setup scripts directly in the ENTRYPOINT
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros_workspace/install/setup.bash && exec \"$@\"", "bash"]

# Use CMD to specify a default action
# CMD ["bash"]
