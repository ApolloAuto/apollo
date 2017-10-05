FROM aarch64/ubuntu:16.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y build-essential
RUN apt-get install -y apt-utils
RUN apt-get install -y curl
RUN apt-get install -y debconf-utils
RUN apt-get install -y doxygen
RUN apt-get install -y lcov
RUN apt-get install -y libboost-all-dev
RUN apt-get install -y libcurl4-openssl-dev
RUN apt-get install -y libfreetype6-dev
RUN apt-get install -y lsof
RUN apt-get install -y python-pip
RUN apt-get install -y python-matplotlib
RUN apt-get install -y python-scipy
RUN apt-get install -y python-software-properties
RUN apt-get install -y realpath
RUN apt-get install -y software-properties-common
RUN apt-get install -y unzip
RUN apt-get install -y vim
RUN apt-get install -y nano
RUN apt-get install -y wget
RUN apt-get install -y zip
RUN apt-get install -y cppcheck
RUN apt-get install -y libgtest-dev
RUN apt-get install -y git
RUN apt-get install -y bc
RUN apt-get install -y apt-transport-https
RUN apt-get install -y shellcheck
RUN apt-get install -y cmake
RUN apt-get install -y gdb
RUN apt-get install -y psmisc
RUN apt-get install -y python-empy
RUN apt-get install -y librosconsole0d
RUN apt-get install -y librosconsole-dev
RUN apt-get install -y libtf-conversions0d

RUN add-apt-repository ppa:webupd8team/java
RUN apt-get update
RUN echo "oracle-java8-installer shared/accepted-oracle-license-v1-1 select true" | debconf-set-selections
RUN apt-get install -y oracle-java8-installer
RUN apt-get clean autoclean && apt-get autoremove -y
RUN rm -fr /var/lib/apt/lists/*

# Install protobuf 3.3.0
WORKDIR /tmp
RUN wget https://github.com/google/protobuf/releases/download/v3.3.0/protobuf-cpp-3.3.0.tar.gz
RUN tar xzf protobuf-cpp-3.3.0.tar.gz
WORKDIR /tmp/protobuf-3.3.0
RUN ./configure --prefix=/usr
RUN make
RUN make install
RUN chmod 755 /usr/bin/protoc

# Set up node v8.0.0
WORKDIR /tmp
RUN wget https://github.com/tj/n/archive/v2.1.0.tar.gz
RUN tar xzf v2.1.0.tar.gz
WORKDIR /tmp/n-2.1.0
RUN make install
RUN n 8.0.0

## Install required python packages.
WORKDIR /tmp
COPY ./modules/tools/py27_requirements.txt /tmp/
RUN pip install -r py27_requirements.txt

# Install yarn
RUN curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | apt-key add -
RUN echo "deb https://dl.yarnpkg.com/debian/ stable main" | tee /etc/apt/sources.list.d/yarn.list
RUN apt-get update && apt-get install -y yarn

ENV ROSCONSOLE_FORMAT '${file}:${line} ${function}() [${severity}] [${time}]: ${message}'

# Install dependency for ros build
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

RUN apt-get update
RUN apt-get install -y libbz2-dev
RUN apt-get install -y libconsole-bridge-dev
RUN apt-get install -y liblog4cxx10-dev
RUN apt-get install -y libeigen3-dev
RUN apt-get install -y liblz4-dev
RUN apt-get install -y libpoco-dev
RUN apt-get install -y libproj-dev
RUN apt-get install -y libtinyxml-dev
RUN apt-get install -y libyaml-cpp-dev
RUN apt-get install -y sip-dev
RUN apt-get install -y uuid-dev
RUN apt-get install -y zlib1g-dev

## https://stackoverflow.com/questions/25193161/chfn-pam-system-error-intermittently-in-docker-hub-builds
RUN ln -s -f /bin/true /usr/bin/chfn

# Install pcl and opencv, prerequisites for Caffe (CPU_ONLY mode)
RUN apt-get update
RUN apt-get install -y libatlas-base-dev
RUN apt-get install -y libflann-dev
RUN apt-get install -y libhdf5-serial-dev
RUN apt-get install -y libicu-dev
RUN apt-get install -y libleveldb-dev
RUN apt-get install -y liblmdb-dev
RUN apt-get install -y libopencv-dev
RUN apt-get install -y libopenni-dev
RUN apt-get install -y libqhull-dev
RUN apt-get install -y libsnappy-dev
RUN apt-get install -y libvtk5-dev
RUN apt-get install -y libvtk5-qt4-dev
RUN apt-get install -y mpi-default-dev

# Install Opengl
RUN echo "deb http://ppa.launchpad.net/keithw/glfw3/ubuntu trusty main" | tee -a /etc/apt/sources.list.d/fillwave_ext.list
RUN echo "deb-src http://ppa.launchpad.net/keithw/glfw3/ubuntu trusty main" | tee -a /etc/apt/sources.list.d/fillwave_ext.list
RUN apt-get update && apt-get install -y --force-yes libglfw3 libglfw3-dev freeglut3-dev

# Install GLEW
WORKDIR /tmp
RUN wget https://github.com/nigels-com/glew/releases/download/glew-2.0.0/glew-2.0.0.zip
RUN unzip glew-2.0.0.zip
WORKDIR /tmp/glew-2.0.0
RUN make && make install

# Remove all temporary files.
WORKDIR /
RUN rm -fr /tmp/*
