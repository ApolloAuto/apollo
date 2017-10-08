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
   gdb \
   git \
   lcov \
   libboost-all-dev \
   libcurl4-openssl-dev \
   libfreetype6-dev \
   lsof \
   python-matplotlib \
   python-pip \
   python-scipy \
   python-software-properties \
   realpath \
   software-properties-common \
   unzip \
   vim \
   google-perftools \
   wget \
   zip

RUN add-apt-repository ppa:webupd8team/java
RUN echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list
RUN curl https://bazel.build/bazel-release.pub.gpg | sudo apt-key add -
RUN echo "oracle-java8-installer shared/accepted-oracle-license-v1-1 select true" | sudo debconf-set-selections

RUN apt-get update && apt-get install -y bazel oracle-java8-installer

RUN apt-get clean autoclean && apt-get autoremove -y
RUN rm -fr /var/lib/apt/lists/*
COPY ./modules/tools/py27_requirements.txt /tmp/

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
WORKDIR /tmp
RUN pip install -r py27_requirements.txt

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
    libopencv-dev \
    libopenni-dev \
    libqhull-dev \
    libsnappy-dev \
    libvtk5-dev \
    libvtk5-qt4-dev \
    mpi-default-dev

# Install glog
WORKDIR /tmp
RUN wget https://github.com/google/glog/archive/v0.3.5.tar.gz
RUN tar xzf v0.3.5.tar.gz
WORKDIR /tmp/glog-0.3.5
RUN ./configure && make && make install

# Install gflags
WORKDIR /tmp
RUN wget https://github.com/gflags/gflags/archive/v2.2.0.tar.gz
RUN tar xzf v2.2.0.tar.gz
WORKDIR /tmp/gflags-2.2.0
RUN mkdir build
WORKDIR /tmp/gflags-2.2.0/build
RUN CXXFLAGS="-fPIC" cmake .. && make && make install

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

# Remove all temporary files.
RUN rm -fr /tmp/*
