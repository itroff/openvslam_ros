FROM ros:noetic
ENV DEBIAN_FRONTEND noninteractive

# install dependencies via apt
ENV DEBCONF_NOWARNINGS yes
RUN set -x && \
  apt-get update -y -qq && \
  apt-get upgrade -y -qq --no-install-recommends && \
  : "basic dependencies" && \
  apt-get install -y -qq \
    build-essential \
    pkg-config \
    cmake \
    git \
    wget \
    curl \
    tar \
    unzip && \
  : "Pangolin dependencies" && \
  apt-get install -y -qq \
    libglew-dev && \
  : "other dependencies" && \
  apt-get install -y -qq \
    libgoogle-glog-dev \
    libyaml-cpp-dev \ 
    libopencv-dev \
    libeigen3-dev && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

ARG CMAKE_INSTALL_PREFIX=/usr/local
ARG NUM_THREADS=8

ENV CPATH=${CMAKE_INSTALL_PREFIX}/include:${CPATH}
ENV C_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include:${C_INCLUDE_PATH}
ENV CPLUS_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include:${CPLUS_INCLUDE_PATH}
ENV LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${LIBRARY_PATH}
ENV LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/lib:${LD_LIBRARY_PATH}

ENV NVIDIA_VISIBLE_DEVICES ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# DBoW2
ARG DBOW2_COMMIT=687fcb74dd13717c46add667e3fbfa9828a7019f
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/OpenVSLAM-Community/DBoW2.git && \
  cd DBoW2 && \
  git checkout ${DBOW2_COMMIT} && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV DBoW2_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/DBoW2

# Pangolin
ARG PANGOLIN_COMMIT=ad8b5f83222291c51b4800d5a5873b0e90a0cf81
WORKDIR /tmp
RUN set -x && \
  git clone https://github.com/stevenlovegrove/Pangolin.git && \
  cd Pangolin && \
  git checkout ${PANGOLIN_COMMIT} && \
  sed -i -e "193,198d" ./src/utils/file_utils.cpp && \
  mkdir -p build && \
  cd build && \
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX} \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_PANGOLIN_DEPTHSENSE=OFF \
    -DBUILD_PANGOLIN_FFMPEG=OFF \
    -DBUILD_PANGOLIN_LIBDC1394=OFF \
    -DBUILD_PANGOLIN_LIBJPEG=OFF \
    -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
    -DBUILD_PANGOLIN_LIBPNG=OFF \
    -DBUILD_PANGOLIN_LIBREALSENSE=OFF \
    -DBUILD_PANGOLIN_LIBREALSENSE2=OFF \
    -DBUILD_PANGOLIN_LIBTIFF=OFF \
    -DBUILD_PANGOLIN_LIBUVC=OFF \
    -DBUILD_PANGOLIN_LZ4=OFF \
    -DBUILD_PANGOLIN_OPENNI=OFF \
    -DBUILD_PANGOLIN_OPENNI2=OFF \
    -DBUILD_PANGOLIN_PLEORA=OFF \
    -DBUILD_PANGOLIN_PYTHON=OFF \
    -DBUILD_PANGOLIN_TELICAM=OFF \
    -DBUILD_PANGOLIN_TOON=OFF \
    -DBUILD_PANGOLIN_UVC_MEDIAFOUNDATION=OFF \
    -DBUILD_PANGOLIN_V4L=OFF \
    -DBUILD_PANGOLIN_VIDEO=OFF \
    -DBUILD_PANGOLIN_ZSTD=OFF \
    -DBUILD_PYPANGOLIN_MODULE=OFF \
    .. && \
  make -j${NUM_THREADS} && \
  make install && \
  cd /tmp && \
  rm -rf *
ENV Pangolin_DIR=${CMAKE_INSTALL_PREFIX}/lib/cmake/Pangolin

# OpenVSLAM
RUN set -x && \
  : "OpenVSLAM dependencies" && \
  apt-get update -y -qq && \
  apt-get install -y -qq \
    ros-${ROS_DISTRO}-libg2o && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*

RUN set -x && \
  git clone --depth 1 https://github.com/OpenVSLAM-Community/openvslam.git && \
  cd openvslam && \
  mkdir -p build && \
  git submodule update -i --recursive && \
  cd build && \
  CMAKE_PREFIX_PATH=/opt/ros/${ROS_DISTRO}/lib/cmake cmake \
    -DUSE_PANGOLIN_VIEWER=ON \
    -DINSTALL_PANGOLIN_VIEWER=ON \
    -DUSE_SOCKET_PUBLISHER=OFF \
    -DUSE_STACK_TRACE_LOGGER=ON \
    -DBOW_FRAMEWORK=FBoW \
    -DBUILD_TESTS=ON \
    -DBUILD_EXAMPLES=ON \
    .. && \
  make -j${NUM_THREADS} && \
  chmod -R 777 ./* && \
  make install 

# ROS
RUN set -x && \
  apt-get update -y -qq && \
  : "install ROS packages" && \
  apt-get install -y -qq \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-cv-bridge && \
 #    ros-${ROS_DISTRO}-tf2 && ros-${ROS_DISTRO}-tf2-ros && ros-${ROS_DISTRO}-tf2-eigen && ros-${ROS_DISTRO}-tf2-msgs && ros-${ROS_DISTRO}-tf2-geometry-msgs && \
  : "remove cache" && \
  apt-get autoremove -y -qq && \
  rm -rf /var/lib/apt/lists/*
RUN apt-get update -y && apt install -y  ros-${ROS_DISTRO}-tf2  ros-${ROS_DISTRO}-tf2-ros  ros-${ROS_DISTRO}-tf2-eigen  ros-${ROS_DISTRO}-tf2-msgs ros-${ROS_DISTRO}-tf2-geometry-msgs
WORKDIR /openvslam/ros/1
RUN mkdir /openvslam/ros/1/src
COPY . /openvslam/ros/1/src/openvslam_ros
RUN set -x && \
  : "build ROS packages" && \
  bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; \
  catkin_make -j${NUM_THREADS} \
    -DBUILD_WITH_MARCH_NATIVE=OFF \
    -DUSE_PANGOLIN_VIEWER=OFF \
    -DUSE_SOCKET_PUBLISHER=OFF \
    -DUSE_STACK_TRACE_LOGGER=OFF \
    -DBOW_FRAMEWORK=DBoW2"

RUN set -x && \
  sh -c "echo '#'\!'/bin/bash\nset -e\nsource /opt/ros/${ROS_DISTRO}/setup.bash\n  export ROS_MASTER_URI=http://10.0.1.11:11311/\n export ROS_IP=10.0.1.11\n export ROS_HOSTNAME=10.0.1.11\nsource /openvslam/ros/1/devel/setup.bash\nexec \"\$@\"' > /ros_entrypoint.sh" && \
  chmod u+x /ros_entrypoint.sh
WORKDIR /openvslam/ros/1/src/openvslam_ros/
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
