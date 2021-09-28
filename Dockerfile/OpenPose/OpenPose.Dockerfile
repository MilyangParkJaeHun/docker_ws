FROM nvidia/cudagl:11.0-devel-ubuntu20.04 AS base-image
ARG USERNAME=park

# Install prerequisites
RUN apt-get -y update && apt-get -y upgrade && \
    DEBIAN_FRONTEND="noninteractive" TZ="Asia/Seoul" \
    apt-get -y install \
        lsb-release \
        cmake git curl wget \
        sudo pkg-config unzip \
        libjpeg-dev libtiff5-dev libpng-dev \
        libavcodec-dev libavformat-dev libswscale-dev libxvidcore-dev libx264-dev libxine2-dev \
        libv4l-dev v4l-utils \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
        libgtk2.0-dev \
        mesa-utils libgl1-mesa-dri libgtkgl2.0-dev libgtkglext1-dev \
        cmake-qt-gui \
        python3-dev python3-pip && \
        pip3 install numpy

# CUDNN
COPY ./files/cudnn-11.0-linux-x64-v8.0.5.39.tgz /root
RUN cd /root && \
    mkdir -p /usr/local/cuda/include && \
    tar -xzvf cudnn-11.0-linux-x64-v8.0.5.39.tgz && \
    rm cudnn-11.0-linux-x64-v8.0.5.39.tgz && \
    cp cuda/include/cudnn*.h /usr/local/cuda/include && \
    cp -P cuda/lib64/libcudnn* /usr/local/cuda/lib64 && \
    chmod a+r /usr/local/cuda/include/cudnn*.h /usr/local/cuda/lib64/libcudnn* && \
    rm -r /root/cuda

ENV PATH=/usr/local/cuda-11.0/bin:$PATH
ENV LD_LIBRARY_PATH=/usr/local/cuda-$CUDA/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

# Create User
RUN useradd --user-group --system --create-home --no-log-init ${USERNAME} && \
    usermod -aG sudo ${USERNAME} && \
    /bin/bash -c 'echo "export PATH=/usr/local/cuda-11.0/bin:$PATH" >> /home/${USERNAME}/.bashrc' && \
    /bin/bash -c 'echo "export LD_LIBRARY_PATH=/usr/local/cuda-$CUDA/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" >> /home/${USERNAME}/.bashrc'

# Install OpenCV
RUN apt-get -y install python3-dev libopencv-dev && \
    pip3 install numpy opencv-python

# Install Openpose
RUN cd /home/${USERNAME} && \
    git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose.git && \
    cd openpose/ && \
    git submodule update --init --recursive --remote && \
    /bin/bash -c 'source scripts/ubuntu/install_deps.sh' && \
    mkdir -p /home/${USERNAME}/openpose/build && \
    cd /home/${USERNAME}/openpose/build && \
    cmake -DBUILD_PYTHON=ON \
        -DDOWNLOAD_BODY_25_MODEL=ON \
        -DDOWNLOAD_BODY_MPI_MODEL=OFF \
        -DDOWNLOAD_HAND_MODEL=OFF \
        -DDOWNLOAD_FACE_MODEL=OFF .. && \  
    sed -ie 's/set(AMPERE "80 86")/#&/g'  ../cmake/Cuda.cmake && \
    sed -ie 's/set(AMPERE "80 86")/#&/g'  ../3rdparty/caffe/cmake/Cuda.cmake && \
    make -j`nproc` && \
    make install
RUN /bin/bash -c 'echo "export PYTHONPATH=/home/${USERNAME}/openpose/build/python" >> /home/${USERNAME}/.bashrc'

# Install ROS
RUN	sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    apt -y update && \
    DEBIAN_FRONTEND="noninteractive" TZ="Asia/Seoul" \
    apt -y install ros-noetic-desktop && \
    echo "source /opt/ros/noetic/setup.bash" >> /home/${USERNAME}/.bashrc && \
    /bin/bash -c 'source /home/${USERNAME}/.bashrc' &&\
    apt-get -y install \
	python3-rosdep \
	python3-rosinstall \
	python3-rosinstall-generator \
	python3-wstool \
	build-essential && \
    rosdep init && \
    rosdep update

RUN apt-get clean && \
	rm -rf /var/lib/apt/lists/*

USER ${USERNAME}
RUN mkdir -p ~/catkin_ws/src/ && \
    /bin/bash -c '. /opt/ros/noetic/setup.bash; /opt/ros/noetic/bin/catkin_make -C ~/catkin_ws' && \
    /bin/bash -c 'echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc' && \
    /bin/bash -c 'echo "export XDG_RUNTIME_DIR=/run/user/1000" >> ~/.bashrc'

WORKDIR /home/${USERNAME}

CMD ["/bin/bash"]
