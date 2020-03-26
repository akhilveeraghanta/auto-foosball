FROM balenalib/raspberrypi3-debian

#######################################################################
#                             Install ROS                             #
#######################################################################
RUN apt-get update
RUN apt-get install lsb-release

RUN echo \
    "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > \
    /etc/apt/sources.list.d/ros-latest.list

RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 \
                --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get install -y \
    python-rosdep \
    python-rosinstall-generator \
    python-wstool \
    python-rosinstall \
    build-essential \
    cmake

RUN rosdep init && rosdep update

RUN rosinstall_generator ros_comm \
        --rosdistro kinetic --deps \
        --wet-only --tar > kinetic-ros_comm-wet.rosinstall
RUN wstool init src kinetic-ros_comm-wet.rosinstall

RUN mkdir -p /app
RUN rosdep install -y --from-paths src \
        --ignore-src --rosdistro kinetic -r --os=debian:buster

RUN apt remove libboost1.67-dev
RUN apt autoremove
RUN apt install -y libboost1.58-dev libboost1.58-all-dev
RUN apt install -y g++-5 gcc-5
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 10
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-5 20
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 10
RUN update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-5 20
RUN update-alternatives --install /usr/bin/cc cc /usr/bin/gcc 30
RUN update-alternatives --set cc /usr/bin/gcc
RUN update-alternatives --install /usr/bin/c++ c++ /usr/bin/g++ 30
RUN update-alternatives --set c++ /usr/bin/g++
RUN apt install -y python-rosdep python-rosinstall-generator\
        python-wstool python-rosinstall build-essential cmake
RUN apt install -y python-empy

RUN ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release\
    --install-space /opt/ros/kinetic -j2

RUN cd /app && rosinstall_generator ros_comm ros_control joystick_drivers --rosdistro kinetic --deps --wet-only --tar > kinetic-custom_ros.rosinstall

RUN wstool merge -t src kinetic-custom_ros.rosinstall
RUN wstool update -t src

RUN rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r --os=debian:jessie

RUN bash -c "source /opt/ros/kinetic/setup.bash"
#######################################################################
#                             Setup Code                              #
#######################################################################
COPY ./src /app/src
RUN cd /app/src && catkin_make
CMD ["bash"]
