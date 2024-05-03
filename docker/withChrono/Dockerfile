FROM uwsbel/ros:latest

# Environmental variables
ENV DISPLAY=:1 \
    VNC_PORT=5901 \
    NO_VNC_PORT=6901 \
    HOME=/sbel \
    TERM=xterm \
    STARTUPDIR=/dockerstartup \
    NO_VNC_HOME=/sbel/noVNC \
    DEBIAN_FRONTEND=noninteractive \
    VNC_COL_DEPTH=24 \
    VNC_RESOLUTION=1600x900 \
    VNC_PW=sbel
EXPOSE $VNC_PORT $NO_VNC_PORT

RUN apt update && apt install -y locales && locale-gen "en_US.UTF-8"
ENV LANG='en_US.UTF-8' LANGUAGE='en_US:en' LC_ALL='en_US.UTF-8'

SHELL ["/bin/bash", "-c"]

# Delete existing Chrono in iros2024
RUN rm -rf /sbel/Desktop/chrono

# Build Chrono and Install
ADD buildChrono.sh /
RUN chmod +x /buildChrono.sh && bash /buildChrono.sh

# Startup and Cleanup
ADD ./src/ $HOME/src/
ADD ./desktop/ $HOME/Desktop/
ADD bashrc $HOME/.bashrc
RUN chmod a+x $HOME/src/vnc_startup.sh $HOME/src/wm_startup.sh \
    && rm -rf /usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1 /usr/lib/x86_64-linux-gnu/libcuda.so.1 /usr/lib/x86_64-linux-gnu/libcudadebugger.so.1

WORKDIR /sbel

# Install Ceres Solver 2.0.0
RUN apt-get update && apt-get install -y cmake libgoogle-glog-dev \
    && wget http://ceres-solver.org/ceres-solver-2.0.0.tar.gz \
    && tar zxf ceres-solver-2.0.0.tar.gz -C /tmp/ \
    && cd /tmp/ceres-solver-2.0.0 \
    && mkdir build && cd build \
    && cmake -DEXPORT_BUILD_DIR=ON \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DUSE_CUDA=ON \
    -DBUILD_TESTING=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DTBB=OFF \
    ../ \
    && make -j $(nproc) \
    && make install -j $(nproc) \
    && rm -rf /tmp/ceres-solver-2.0.0 /tmp/ceres-solver-2.0.0.tar.gz

# Install OpenCV 4.2, Eigen 3, G2O, Boost, Glog, and ONNX
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libboost-all-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    libqt5core5a \
    libqt5gui5 \
    libqt5widgets5 \
    libqt5opengl5-dev \
    libgl1-mesa-dev \
    libqglviewer-dev-qt5 \
    && pip install opencv-python onnx \
    && git clone https://github.com/RainerKuemmerle/g2o.git /tmp/g2o \
    && cd /tmp/g2o \
    && git checkout 20230223_git \
    && mkdir build && cd build \
    && cmake .. -DG2O_HAVE_CHOLMOD=ON -DG2O_BUILD_VIEWER=ON \
    && make -j $(nproc) \
    && make install
    # && rm -rf /tmp/g2o

# Add NVIDIA package repositories
# Add NVIDIA package repositories
# RUN apt-get update && apt-get install -y wget software-properties-common gnupg2 \
#     && wget -qO - https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/3bf863cc.pub | apt-key add - \
#     && wget -qO - https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu2204/x86_64/7fa2af80.pub | apt-key add - \
#     && echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/ /" > /etc/apt/sources.list.d/nvidia-cuda.list \
#     && echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu2204/x86_64/ /" > /etc/apt/sources.list.d/nvidia-ml.list

# Install TensorRT and CUDA Toolkit dependencies
RUN apt-get update && apt-get install -y \
    tensorrt \
    libcudnn8 \
    libcudnn8-dev

# Install TensorRT and CUDA Toolkit dependencies
RUN apt-get update && apt-get install -y \
    tensorrt \
    libcudnn8 \
    libcudnn8-dev


# Install ROS2 Humble
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
    && apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
    && apt-get update && apt-get install -y ros-humble-desktop

# Clone AirVo
# RUN mkdir ros2_ws && cd ros2_ws \
#     && mkdir src && cd src \
#     && git clone -b ros2 https://github.com/srirama02/AirVO.git

RUN mkdir -p /sbel/ros2_ws/src


WORKDIR /sbel
ENTRYPOINT ["/sbel/src/vnc_startup.sh"]
