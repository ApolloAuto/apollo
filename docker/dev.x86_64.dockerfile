FROM ubuntu:14.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
   apt-transport-https \
   bc \
   build-essential \
   cmake \
   cppcheck \
   curl \
   debconf-utils \
   doxygen \
   graphviz \
   gdb \
   gfortran \
   git \
   subversion \
   google-perftools \
   lcov \
   libblas-dev \
   libboost-all-dev \
   libcurl4-openssl-dev \
   libfreetype6-dev \
   libgeos-dev \
   liblapack-dev \
   libleveldb-dev \
   lsof \
   python-matplotlib \
   python-pip \
   python-scipy \
   python-software-properties \
   realpath \
   software-properties-common \
   unzip \
   vim \
   v4l-utils \
   nfs-common \
   wget \
   zip

# Create a soft link for libprofiler
RUN ln -rs /usr/lib/libprofiler.so.0 /usr/lib/libprofiler.so
RUN ln -rs /usr/lib/libtcmalloc_and_profiler.so.4 /usr/lib/libtcmalloc_and_profiler.so

RUN add-apt-repository -y ppa:webupd8team/java
RUN add-apt-repository -y ppa:gluster/glusterfs-3.10
RUN echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list
RUN curl https://bazel.build/bazel-release.pub.gpg | sudo apt-key add -
RUN echo "oracle-java8-installer shared/accepted-oracle-license-v1-1 select true" | sudo debconf-set-selections

RUN apt-get update && apt-get install -y \
    glusterfs-client \
    oracle-java8-installer

WORKDIR /tmp
RUN wget https://github.com/bazelbuild/bazel/releases/download/0.5.3/bazel-0.5.3-installer-linux-x86_64.sh
RUN bash bazel-0.5.3-installer-linux-x86_64.sh

RUN apt-get clean autoclean && apt-get autoremove -y
RUN rm -fr /var/lib/apt/lists/*

WORKDIR /tmp
# install protobuf 3.3.0
RUN wget https://github.com/google/protobuf/releases/download/v3.3.0/protobuf-cpp-3.3.0.tar.gz
RUN tar xzf protobuf-cpp-3.3.0.tar.gz
WORKDIR /tmp/protobuf-3.3.0
RUN ./configure --prefix=/usr && make && make install
RUN chmod 755 /usr/bin/protoc

# Set up node v8.0.0
WORKDIR /tmp
RUN wget https://github.com/tj/n/archive/v2.1.0.tar.gz
RUN tar xzf v2.1.0.tar.gz
WORKDIR /tmp/n-2.1.0
RUN make install
RUN n 8.0.0

# Install required python packages.
COPY docker/py27_requirements.txt /tmp/
RUN pip install -r /tmp/py27_requirements.txt

# Install yarn
RUN curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
RUN echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
RUN apt-get update && apt-get install -y yarn

ENV ROSCONSOLE_FORMAT '${file}:${line} ${function}() [${severity}] [${time}]: ${message}'

# Install dependency for ros build
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update && apt-get install -y \
    libbz2-dev \
    libconsole-bridge-dev \
    libeigen3-dev \
    libgstreamer-plugins-base0.10-dev \
    libgstreamer0.10-dev \
    liblog4cxx10-dev \
    liblz4-dev \
    libpoco-dev \
    libproj-dev \
    libtinyxml-dev \
    libyaml-cpp-dev \
    ros-indigo-catkin \
    sip-dev \
    uuid-dev \
    zlib1g-dev

RUN add-apt-repository "deb http://archive.ubuntu.com/ubuntu trusty-backports universe"
RUN apt-get update && apt-get install shellcheck

# https://stackoverflow.com/questions/25193161/chfn-pam-system-error-intermittently-in-docker-hub-builds
RUN ln -s -f /bin/true /usr/bin/chfn

# Install pcl and opencv, prerequisites for Caffe (CPU_ONLY mode)
RUN apt-get update && apt-get install -y \
    libatlas-base-dev \
    libflann-dev \
    libhdf5-serial-dev \
    libicu-dev \
    libleveldb-dev \
    liblmdb-dev \
    libopenblas-dev \
    libopencv-dev \
    libopenni-dev \
    libqhull-dev \
    libsnappy-dev \
    libvtk5-dev \
    libvtk5-qt4-dev \
    mpi-default-dev

# Install gflags
WORKDIR /tmp
RUN wget https://github.com/gflags/gflags/archive/v2.2.0.tar.gz
RUN tar xzf v2.2.0.tar.gz
WORKDIR /tmp/gflags-2.2.0
RUN mkdir build
WORKDIR /tmp/gflags-2.2.0/build
RUN CXXFLAGS="-fPIC" cmake .. && make && make install

# Install glog
WORKDIR /tmp
RUN wget https://github.com/google/glog/archive/v0.3.5.tar.gz
RUN tar xzf v0.3.5.tar.gz
WORKDIR /tmp/glog-0.3.5
RUN ./configure
RUN make CXXFLAGS='-Wno-sign-compare -Wno-unused-local-typedefs -D_START_GOOGLE_NAMESPACE_="namespace google {" -D_END_GOOGLE_NAMESPACE_="}" -DGOOGLE_NAMESPACE="google" -DHAVE_PTHREAD -DHAVE_SYS_UTSNAME_H -DHAVE_SYS_SYSCALL_H -DHAVE_SYS_TIME_H -DHAVE_STDINT_H -DHAVE_STRING_H -DHAVE_PREAD -DHAVE_FCNTL -DHAVE_SYS_TYPES_H -DHAVE_SYSLOG_H -DHAVE_LIB_GFLAGS -DHAVE_UNISTD_H'
RUN make install

# Install Opengl
RUN echo "deb http://ppa.launchpad.net/keithw/glfw3/ubuntu trusty main" | sudo tee -a /etc/apt/sources.list.d/fillwave_ext.list
RUN echo "deb-src http://ppa.launchpad.net/keithw/glfw3/ubuntu trusty main" | sudo tee -a /etc/apt/sources.list.d/fillwave_ext.list
RUN apt-get update && apt-get install -y --force-yes libglfw3 libglfw3-dev freeglut3-dev

# Install GLEW
WORKDIR /tmp
RUN wget https://github.com/nigels-com/glew/releases/download/glew-2.0.0/glew-2.0.0.zip
RUN unzip glew-2.0.0.zip
WORKDIR /tmp/glew-2.0.0
RUN make && make install
RUN ln -s /usr/lib64/libGLEW.so /usr/lib/libGLEW.so
RUN ln -s /usr/lib64/libGLEW.so.2.0 /usr/lib/libGLEW.so.2.0

# install pcl
WORKDIR /tmp/
RUN wget https://apollocache.blob.core.windows.net/apollo-docker/pcl-1.7_x86.tar.gz
RUN tar xzf pcl-1.7_x86.tar.gz
RUN mv pcl-1.7_x86/include/pcl /usr/local/include/pcl-1.7/
RUN mv pcl-1.7_x86/lib/* /usr/local/lib/
RUN mv pcl-1.7_x86/share/pcl-1.7 /usr/local/share/
RUN ldconfig

# install nvidia driver
WORKDIR /tmp/
RUN wget https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda_8.0.61_375.26_linux-run
RUN chmod +x cuda_8.0.61_375.26_linux-run
RUN ./cuda_8.0.61_375.26_linux-run -silent --toolkit --driver

# install cuda
WORKDIR /tmp/
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1404/x86_64/cuda-repo-ubuntu1404_8.0.61-1_amd64.deb
RUN dpkg -i cuda-repo-ubuntu1404_8.0.61-1_amd64.deb
RUN apt-get update && apt-get install -y cuda

# let users install cudnn 7.0 by themselves

# install gpu_caffe
WORKDIR /tmp
RUN wget https://apollocache.blob.core.windows.net/apollo-docker/caffe_x86.tar.gz
RUN tar xzf caffe_x86.tar.gz
RUN mv caffe_x86/output-GPU/include/caffe /usr/include/
RUN mv caffe_x86/output-GPU/lib/* /usr/lib/x86_64-linux-gnu/
RUN ldconfig

# install ipopt
RUN apt-get install libblas-dev liblapack-dev gfortran
WORKDIR /tmp
RUN wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.8.zip -O Ipopt-3.12.8.zip
RUN unzip Ipopt-3.12.8.zip
RUN cd Ipopt-3.12.8/ThirdParty/Mumps && bash get.Mumps && cd ../../
WORKDIR /tmp/Ipopt-3.12.8
RUN ./configure --build=x86_64
RUN make all && make install && mkdir -p /usr/local/ipopt
RUN cp -r include /usr/local/ipopt/ && cp -r lib /usr/local/ipopt/

# install qp_oases
WORKDIR /tmp
RUN wget https://github.com/startcode/qp-oases/archive/v3.2.1-1.tar.gz
RUN tar xzf v3.2.1-1.tar.gz
WORKDIR qp-oases-3.2.1-1
RUN mkdir bin && make CPPFLAGS="-Wall -pedantic -Wshadow -Wfloat-equal -O3 -Wconversion  -Wsign-conversion -fPIC -DLINUX -DSOLVER_NONE -D__NO_COPYRIGHT__"
RUN cp bin/libqpOASES.so /usr/local/lib
RUN cp -r include/* /usr/local/include
RUN ldconfig

# add ros
RUN mkdir /home/tmp
WORKDIR /home/tmp
# RUN wget https://github.com/ApolloAuto/apollo-platform/releases/download/1.5.4/ros-indigo-apollo-1.5.4-x86_64.tar.gz
RUN wget https://github.com/ApolloAuto/apollo-platform/releases/download/2.0.0/ros-indigo-apollo-2.0.0-x86_64.tar.gz
RUN tar xzf ros-indigo-apollo-2.0.0-x86_64.tar.gz
RUN rm ros-indigo-apollo-2.0.0-x86_64.tar.gz

# install nlopt
WORKDIR /tmp
RUN wget http://ab-initio.mit.edu/nlopt/nlopt-2.4.2.tar.gz
RUN tar xzf nlopt-2.4.2.tar.gz
WORKDIR /tmp/nlopt-2.4.2
RUN ./configure --enable-shared && make && make install
RUN rm /usr/local/lib/libnlopt.a
RUN rm /usr/local/lib/libnlopt.la

# setup OTA security package
WORKDIR /tmp
RUN wget https://apollocache.blob.core.windows.net/apollo-docker/ota_security.tar.gz
RUN tar xzf ota_security.tar.gz
WORKDIR /tmp/ota_security
RUN bash ota_server_deploy.sh root

# add supervisord config file
RUN echo_supervisord_conf > /etc/supervisord.conf

# add undistort lib and include
WORKDIR /tmp
RUN wget https://apollocache.blob.core.windows.net/apollo-docker/undistort.tar.gz
RUN tar xzf undistort.tar.gz
RUN mkdir -p /usr/local/apollo
RUN mv undistort /usr/local/apollo/

# Add snowboy library.
# First we need to upgrade libstdc++.so.6.0.19 to libstdc++.so.6.0.24 to be
# compatible with the library.
RUN add-apt-repository -y ppa:ubuntu-toolchain-r/test && \
    apt-get update -y && \
    apt-get install -y --only-upgrade libstdc++
WORKDIR /tmp
RUN wget https://apollocache.blob.core.windows.net/apollo-docker/snowboy.tar.gz
RUN tar xzf snowboy.tar.gz
RUN mkdir -p /usr/local/apollo
RUN mv snowboy /usr/local/apollo/

RUN rm -rf /tmp/*
