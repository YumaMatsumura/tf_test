# Copyright 2024 Yuma Matsumura All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

FROM osrf/ros:jazzy-desktop

LABEL maintainer="yumapine@gmail.com"

ENV ROS_DISTRO=jazzy

# Set environment variables
ENV ROS_DOMAIN_ID=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Update and clean up apt lists
RUN apt update && \
    rm -rf /var/lib/apt/lists/*

# Install necessary ROS 2 packages and colcon build tools
RUN apt update && \
    apt install -y ros-$ROS_DISTRO-tf2 \
                   ros-$ROS_DISTRO-tf2-ros \
                   ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
                   python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# Create the workspace directory
RUN mkdir -p ~/ros2_ws/src

# Set the working directory
WORKDIR /root/ros2_ws

# Copy the local directory (where the Dockerfile is located) into the ROS 2 workspace src directory
COPY . /root/ros2_ws/src/

# Source the ROS 2 setup script and build the workspace using colcon
RUN /bin/bash -c '. /opt/ros/jazzy/setup.bash; colcon build'

# Copy the entrypoint script into the container and make it executable
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# Set the default command to bash
CMD ["bash"]
