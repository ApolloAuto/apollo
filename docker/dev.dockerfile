FROM ubuntu:14.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
   apt-transport-https \
   bc \
   build-essential \
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
   python-pip \
   python-matplotlib \
   python-scipy \
   python-software-properties \
   realpath \
   software-properties-common \
   unzip \
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
# install protobuf 3.1.0
RUN wget https://github.com/google/protobuf/releases/download/v3.1.0/protobuf-cpp-3.1.0.tar.gz
RUN tar xzf protobuf-cpp-3.1.0.tar.gz
WORKDIR /tmp/protobuf-3.1.0
RUN ./configure --prefix=/usr
RUN make
RUN make install

WORKDIR /tmp
RUN wget https://github.com/google/protobuf/releases/download/v3.1.0/protoc-3.1.0-linux-x86_64.zip
RUN unzip protoc-3.1.0-linux-x86_64.zip -d protoc3
RUN mv protoc3/bin/protoc /usr/bin/
RUN chmod 755 /usr/bin/protoc

# set up node v8.0.0
RUN wget https://github.com/tj/n/archive/v2.1.0.tar.gz
RUN tar xzf v2.1.0.tar.gz
WORKDIR /tmp/n-2.1.0
RUN make install
RUN n 8.0.0

WORKDIR /tmp
# Install required python packages.
RUN pip install -r py27_requirements.txt

# Install yarn
RUN curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
RUN echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
RUN apt-get update && apt-get install -y yarn

# Remove all temporary files.
RUN rm -fr /tmp/*

ENV ROSCONSOLE_FORMAT '${file}:${line} ${function}() [${severity}] [${time}]: ${message}'

# install dependency for ros build
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN apt-get update && apt-get install -y \
    ros-indigo-catkin \
    libbz2-dev \
    libconsole-bridge-dev \
    liblog4cxx10-dev \
    libeigen3-dev \
    liblz4-dev \
    libpoco-dev \
    libproj-dev \
    libtinyxml-dev \
    libyaml-cpp-dev \
    sip-dev \
    uuid-dev \
    zlib1g-dev

RUN add-apt-repository "deb http://archive.ubuntu.com/ubuntu trusty-backports universe"
RUN apt-get update && apt-get install shellcheck

# https://stackoverflow.com/questions/25193161/chfn-pam-system-error-intermittently-in-docker-hub-builds
RUN ln -s -f /bin/true /usr/bin/chfn
