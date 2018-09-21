FROM ubuntu:bionic
# 1. Install dependencies
RUN apt-get update -y
RUN apt-get install -qq -y \
	g++ \
	git \
	python-pip \
	python-empy \
	cmake \
	libtinyxml2-dev \
	libeigen3-dev \
	libxml2-dev \
	libboost-all-dev \
	libgtest-dev
	
RUN pip install -U pip catkin-tools
# 2. Install Gazebo 9
RUN curl -sSL http://get.gazebosim.org | sh
# 3. Create ROS tools workspace
RUN mkdir -p $HOME/ws/ros_tools_ws/src
WORKDIR $HOME/ws/ros_tools_ws/src
# 3.1 Download the packages
RUN git clone https://github.com/ros/catkin
RUN git clone https://github.com/ros/rospack
RUN git clone https://github.com/ros/ros
RUN git clone https://github.com/ros/cmake_modules
# 3.2 Configure the workspace
WORKDIR $HOME/ws/ros_tools_ws
RUN catkin config --init --install --cmake-args -DCMAKE_BUILD_TYPE=Release
# 3.3 Build the workspace
RUN catkin build
# 4. Create the ORCA workspace
RUN mkdir -p $HOME/ws/orca_ws/src
WORKDIR $HOME/ws/orca_ws/src
# 4.1 Download the packages
RUN git clone https://github.com/syroco/orca
# 4.2 Configure the workspace
WORKDIR $HOME/ws/orca_ws
RUN catkin config --init --extend /ws/ros_tools_ws/install --install --cmake-args -DCMAKE_BUILD_TYPE=Release
# 4.3 Build the workspace
RUN catkin build --summarize
