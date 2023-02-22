FROM ubuntu:focal

# Expose port 5000, this used by the web interface of RMF
# not necessary for benchmark
#EXPOSE 5000

# Use bash as default shell
SHELL ["/bin/bash", "-c"]

# install ROS and some basic packages
RUN \
    apt-get -y -q update && \
    # Prevents debconf from prompting for user input
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    curl gnupg2 lsb-release \
    gcc \
    g++ \
    build-essential \
    cmake \
    wget \
    curl \
    unzip \
    git \
    git-lfs \
    cmake \
    python3-pip \
    htop \
    vim -y  


# Create ROS workspace
ENV MRP_BENCH_ROOT=/mrp_bench
ENV REPO_NAME=mrp_bench
WORKDIR $MRP_BENCH_ROOT

# Add our own ROS package(s) to Docker image
# NOTE: At design time, this could be overridden by a volume mount to be able to edit the code
# from outside of the container, and commit these changes to git.
COPY . ${MRP_BENCH_ROOT}/



RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN \
    apt-get -y -q update && \
    # Prevents debconf from prompting for user input
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-galactic-ros-base

# Install python related dependecies for RMF
RUN DEBIAN_FRONTEND=noninteractive apt install \
    apt-utils \
    python3-rosdep \
    python3-vcstool \
    python3-shapely python3-yaml python3-requests \
    python3-colcon-* npm \
    -y

RUN python3 -m pip install Flask flask-socketio flask-cors

# install extra python packages
RUN python3 -m pip install --upgrade --user ortools dotmap networkx

# Install Nav2
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-galactic-navigation2 ros-galactic-nav2-bringup ros-galactic-nav2-gazebo-spawner ros-galactic-turtlebot3*


# install libMultiRobotPlanning
WORKDIR /
RUN git clone --depth 1 https://github.com/whoenig/libMultiRobotPlanning.git \
    && mkdir /libMultiRobotPlanning/build \
    && cd /libMultiRobotPlanning/build \
    && cmake .. && make

# install EECBS
RUN DEBIAN_FRONTEND=noninteractive apt install libboost-all-dev -y
WORKDIR /
RUN git clone --depth 1 https://github.com/Jiaoyang-Li/EECBS.git \
    && cd /EECBS \
    && cmake -DCMAKE_BUILD_TYPE=RELEASE . \
    && make

# Install RMF from source
# clone repos
RUN mkdir -p $MRP_BENCH_ROOT/rmf_ws/src
WORKDIR $MRP_BENCH_ROOT/rmf_ws
RUN wget https://raw.githubusercontent.com/open-rmf/rmf/galactic/rmf.repos && vcs import src < rmf.repos \ 
    && git clone --depth 1 https://github.com/open-rmf/free_fleet.git
WORKDIR $MRP_BENCH_ROOT/rmf_ws
RUN rosdep init && rosdep update --include-eol-distros && rosdep install --from-paths src --ignore-src --rosdistro galactic -y


# download pre-traced maps
RUN wget -o /dev/null -O $MRP_BENCH_ROOT/$REPO_NAME/launch/mrp_bench_maps/maps/maps.zip "https://bwsyncandshare.kit.edu/s/c83NZ6QoHT3Btoo/download"
WORKDIR $MRP_BENCH_ROOT/$REPO_NAME/launch/mrp_bench_maps/maps
RUN unzip $MRP_BENCH_ROOT/$REPO_NAME/launch/mrp_bench_maps/maps/maps.zip \
    && touch $MRP_BENCH_ROOT/$REPO_NAME/map.pgm \
    && mv office.pgm office \
    && mv airport_terminal.pgm airport_terminal \
    && mv warehouse.pgm warehouse


# prepare env
RUN mkdir /experiments
RUN mv $MRP_BENCH_ROOT/$REPO_NAME/bench_ws/bench_pkg/bench_pkg/param/docker_config.yaml $MRP_BENCH_ROOT/$REPO_NAME/bench_ws/bench_pkg/bench_pkg/param/config.yaml 
RUN ln -s $MRP_BENCH_ROOT/rmf_ws $MRP_BENCH_ROOT/$REPO_NAME/rmf_ws

# selecting right version of nlohmann_json_schema_validator_vendor for galactic
WORKDIR  $MRP_BENCH_ROOT/rmf_ws/src/thirdparty/nlohmann_json_schema_validator_vendor/
RUN git checkout f5fca6cdb98748e372d519f3ab2601e14fa30c06

# build
WORKDIR $MRP_BENCH_ROOT/$REPO_NAME
RUN source /opt/ros/galactic/setup.bash && colcon build 


# setup bashrc
RUN echo "alias s='source /opt/ros/galactic/setup.bash && source /$MRP_BENCH_ROOT/$REPO_NAME/install/setup.bash'" >> /root/.bashrc && \
    echo "alias b='cd $MRP_BENCH_ROOT/$REPO_NAME'" >> /root/.bashrc && \
    echo "alias config='vi ./install/bench_pkg/share/bench_pkg/param/config.yaml'" && \
    echo "alias ..='cd ..'"

WORKDIR /
# define aliases for running experiment 








