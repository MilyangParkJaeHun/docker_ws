FROM nvidia/cuda:11.2.2-cudnn8-devel-ubuntu20.04 AS build-image
ARG USERNAME=deepws

ARG GRAPHIC_CARD_ARCH_VER=7.5
ARG CUDA_VER=10.2
ARG CUDNN_VER=7.6.5
ARG CUDNN_VER_MAIN=7
ARG TRT_VER=6.0.1

RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub


RUN apt-get update -y && apt-get -y upgrade && \
  DEBIAN_FRONTEND="noninteractive" TZ="Asia/Seoul" \
  apt-get -y install \
    apt-utils lsb-release \
    cmake git curl wget unzip \
    sudo pkg-config build-essential\
    python3-pip python3-dev && \
  pip3 install pip --upgrade

RUN useradd -ms /bin/bash ${USERNAME}

# [Set CUDA env]
RUN export PATH=/usr/local/cuda-$CUDA_VER/bin:$PATH && \
  export LD_LIBRARY_PATH=/usr/local/cuda-$CUDA_VER/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# [Install OpenCV]
RUN apt-get -y install \
  build-essential cmake pkg-config unzip yasm git checkinstall \
  libjpeg-dev libpng-dev libtiff-dev \
  libavcodec-dev libavformat-dev libswscale-dev libavresample-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  libxvidcore-dev x264 libx264-dev libfaac-dev libmp3lame-dev libtheora-dev \
  libfaac-dev libmp3lame-dev libvorbis-dev \
  libopencore-amrnb-dev libopencore-amrwb-dev \
  libdc1394-22 libdc1394-22-dev libxine2-dev libv4l-dev v4l-utils \
  libgtk-3-dev \
  python3-testresources \
  libtbb-dev \
  libatlas-base-dev gfortran \
  libprotobuf-dev protobuf-compiler \
  libgoogle-glog-dev libgflags-dev \
  libgphoto2-dev libeigen3-dev libhdf5-dev doxygen

RUN pip3 install -U pip numpy

RUN mkdir -p ~/opencv_build && \
  cd ~/opencv_build && \
  wget -O opencv.zip https://github.com/opencv/opencv/archive/refs/tags/4.5.5.zip && \
  unzip opencv.zip && \
  wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/refs/tags/4.5.5.zip && \
  unzip opencv_contrib.zip

RUN apt-get -y install software-properties-common && \
  add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
  apt-get -y update && \
  apt-get install -y gcc-snapshot

RUN mkdir -p ~/opencv_build/opencv-4.5.5/build && \
  cd ~/opencv_build/opencv-4.5.5/build && \
  cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D WITH_TBB=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D WITH_CUDA=ON \
    -D BUILD_opencv_cudacodec=OFF \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D CUDA_ARCH_BIN=7.5 \
    -D WITH_V4L=ON \
    -D WITH_QT=OFF \
    -D WITH_OPENGL=ON \
    -D WITH_GSTREAMER=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_PC_FILE_NAME=opencv.pc \
    -D OPENCV_ENABLE_NONFREE=OFF \
    -D BUILD_NEW_PYTHON_SUPPORT=ON \
    -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib-4.5.5/modules \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D BUILD_EXAMPLES=OFF .. && \
  make -j$(nproc) && \
  sudo make install && \
  rm -rf ~/opencv_build

RUN apt-get clean && \
	rm -rf /var/lib/apt/lists/*

USER ${USERNAME}

RUN /bin/bash -c 'echo "" >> ~/.bashrc' && \
  /bin/bash -c 'echo "# CUDA" >> ~/.bashrc' && \
  /bin/bash -c 'echo "export PATH=/usr/local/cuda-$CUDA_VER/bin:$PATH" >> ~/.bashrc' && \
  /bin/bash -c 'echo "LD_LIBRARY_PATH=/usr/local/cuda-$CUDA_VER/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" >> ~/.bashrc' && \
  /bin/bash -c 'echo "" >> ~/.bashrc' && \
  /bin/bash -c 'echo "# python" >> ~/.bashrc' && \
  /bin/bash -c 'echo "alias python=python3" >> ~/.bashrc'

WORKDIR /home/${USERNAME}

CMD ["/bin/bash"]
