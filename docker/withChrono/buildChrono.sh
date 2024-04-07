#!/bin/bash
source /opt/ros/humble/setup.sh
source /chrono_ros_interfaces/install/setup.bash

# ------------------------------------------------------------------------
# Vulkan
# ------------------------------------------------------------------------
wget -qO- https://packages.lunarg.com/lunarg-signing-key-pub.asc | tee /etc/apt/trusted.gpg.d/lunarg.asc && \
wget -qO /etc/apt/sources.list.d/lunarg-vulkan-jammy.list http://packages.lunarg.com/vulkan/lunarg-vulkan-jammy.list && \
apt-get update && apt-get install vulkan-sdk -y && \
apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*


git clone -b feature/vio https://github.com/Huzaifg/chrono.git $HOME/Desktop/chrono

cd $HOME/Desktop/chrono/contrib/build-scripts/vsg/ && \
    bash buildVSG.sh /opt/vsg
    
cd $HOME/Desktop/chrono
git submodule init
git submodule update

# ------------------------------------------------------------------------

SOURCE_DIR="$HOME/Desktop/chrono"
BUILD_DIR="$HOME/Desktop/chrono/build"
INSTALL_DIR="$HOME/Desktop/chrono/install"

EIGEN3_INSTALL_DIR="/root/Packages/eigen-3.4.0"
BLAZE_INSTALL_DIR="/root/Packages/blaze-3.8"
SPECTRA_INSTALL_DIR="/root/Packages/spectra"

CRG_INCLUDE_DIR="/root/Packages/OpenCRG/include"
CRG_LIBRARY="/root/Packages/OpenCRG/lib/libOpenCRG.1.1.2.a"

IRRLICHT_INSTALL_DIR="/root/Packages/irrlicht-1.8.5"
VSG_INSTALL_DIR="/opt/vsg"
LIB_DIR="lib"
GL_INSTALL_DIR="/root/Packages/gl"

OPTIX_INSTALL_DIR="/root/Packages/optix-7.7.0"
FASTRTPS_INSTALL_DIR="/root/Packages/fastrtps-2.4.0"

GVDB_INCLUDE="$HOME/Desktop/chrono/src/chrono_thirdparty/gvdb-voxels/_output/include"
GVDB_LIBRARY="$HOME/Desktop/chrono/src/chrono_thirdparty/gvdb-voxels/_output/bin/libgvdb.so"

SWIG_EXE="swig"


# ------------------------------------------------------------------------

BUILDSYSTEM="Ninja"

# ------------------------------------------------------------------------

cmake -G ${BUILDSYSTEM} -B ${BUILD_DIR} -S ${SOURCE_DIR} \
      -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DIR} \
      -DENABLE_MODULE_IRRLICHT:BOOL=OFF \
      -DENABLE_MODULE_VSG:BOOL=ON \
      -DENABLE_MODULE_OPENGL:BOOL=OFF \
      -DENABLE_MODULE_VEHICLE:BOOL=ON \
      -DIOMP5_LIBRARY=${IOMP5_DIR} \
      -DENABLE_MODULE_POSTPROCESS:BOOL=OFF \
      -DENABLE_MODULE_MULTICORE:BOOL=OFF \
      -DENABLE_MODULE_FSI:BOOL=OFF \
      -DENABLE_MODULE_GPU:BOOL=OFF \
      -DENABLE_MODULE_DISTRIBUTED:BOOL=OFF \
      -DENABLE_MODULE_PARDISO_MKL:BOOL=OFF \
      -DENABLE_MODULE_CASCADE:BOOL=OFF \
      -DENABLE_MODULE_COSIMULATION:BOOL=OFF \
      -DENABLE_MODULE_SENSOR:BOOL=ON \
      -DENABLE_OPENMP=ON \
      -DENABLE_MODULE_MODAL:BOOL=OFF \
      -DENABLE_MODULE_MATLAB:BOOL=OFF \
      -DENABLE_MODULE_CSHARP:BOOL=OFF \
      -DENABLE_MODULE_PYTHON:BOOL=OFF \
      -DENABLE_MODULE_SYNCHRONO:BOOL=ON \
      -DENABLE_MODULE_ROS:BOOL=ON \
      -DBUILD_BENCHMARKING:BOOL=OFF \
      -DBUILD_TESTING:BOOL=OFF \
      -DBUILD_DEMOS=ON \
      -DENABLE_OPENCRG:BOOL=OFF \
      -DUSE_CUDA_NVRTC:BOOL=ON \
      -DUSE_FAST_DDS:BOOL=OFF \
      -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INSTALL_DIR} \
      -DBLAZE_INSTALL_DIR:PATH=${BLAZE_INSTALL_DIR} \
      -DOptiX_INSTALL_DIR:PATH=${OPTIX_INSTALL_DIR} \
      -Dfastrtps_INSTALL_DIR:PATH=${FASTRTPS_INSTALL_DIR} \
      -DGLEW_DIR=${GL_INSTALL_DIR}/${LIB_DIR}/cmake/glew \
      -Dglfw3_DIR=${GL_INSTALL_DIR}/${LIB_DIR}/cmake/glfw3 \
      -DGLM_INCLUDE_DIR:PATH=${GL_INSTALL_DIR}/include \
      -DOpenCRG_INCLUDE_DIR:PATH=${CRG_INCLUDE_DIR} \
      -DOpenCRG_LIBRARY:FILEPATH=${CRG_LIBRARY} \
      -DSPECTRA_INCLUDE_DIR:PATH=${SPECTRA_INSTALL_DIR}/include \
      -DMATLAB_SDK_ROOT:PATH=${MATLAB_INSTALL_DIR}/extern \
      -Dvsg_DIR:PATH=${VSG_INSTALL_DIR}/${LIB_DIR}/cmake/vsg \
      -DvsgImGui_DIR:PATH=${VSG_INSTALL_DIR}/${LIB_DIR}/cmake/vsgImGui \
      -DvsgXchange_DIR:PATH=${VSG_INSTALL_DIR}/${LIB_DIR}/cmake/vsgXchange \
      -DSWIG_EXECUTABLE:FILEPATH=${SWIG_EXE} \
      -DCMAKE_BUILD_TYPE="Release" \
      -DUSE_GVDB:BOOL=OFF \
      -DGVDB_INCLUDE:PATH=${GVDB_INCLUDE} \
      -DGVDB_LIBRARY:PATH=${GVDB_LIBRARY} \
      -DCUDA_ARCH_NAME="All"

cd build
ninja -j 16
ninja install
