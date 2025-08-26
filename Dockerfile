# Use the ROS 2 Humble image with desktop tools, including Gazebo
FROM ros:humble-desktop-full

# Set the shell to bash
SHELL ["/bin/bash", "-c"]

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies required for ROS 2 development and this project
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    # Clean up apt cache to keep image size down
    && rm -rf /var/lib/apt/lists/*

# Install ArduPilot SITL dependencies using the official script
# We clone the ardupilot repo, run the script, and then remove the repo to keep the image clean.
RUN git clone --depth 1 https://github.com/ArduPilot/ardupilot.git /ardupilot \
    && cd /ardupilot/Tools/environment_install && ./install-prereqs-ubuntu.sh -y \
    && rm -rf /ardupilot

# Initialize rosdep. This is needed to install ROS package dependencies.
RUN rosdep init && rosdep update

# Create and set the working directory for our ROS 2 workspace
WORKDIR /ros2_ws

# Copy the entire project source code into the workspace's src directory
COPY . ./src

# Install all ROS dependencies for the project
# Sourcing the setup file is necessary to make rosdep and other ROS tools available
RUN . /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y

# Set up the entrypoint script that will configure the shell environment
COPY entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# The default command when running the container is to start an interactive bash shell
CMD ["bash"]
