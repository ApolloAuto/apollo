FROM ubuntu:14.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
   build-essential \
   curl \
   debconf-utils \
   doxygen \
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
   zip \
   cppcheck

RUN add-apt-repository ppa:webupd8team/java
RUN echo "deb [arch=amd64] http://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list
RUN curl https://bazel.build/bazel-release.pub.gpg | sudo apt-key add -
RUN echo "oracle-java8-installer shared/accepted-oracle-license-v1-1 select true" | sudo debconf-set-selections

RUN apt-get update && apt-get install -y bazel oracle-java8-installer

RUN apt-get clean autoclean && apt-get autoremove -y
RUN rm -fr /var/lib/apt/lists/*

#install protobuf 3.1.0
WORKDIR /root
RUN wget https://github.com/google/protobuf/releases/download/v3.1.0/protobuf-cpp-3.1.0.tar.gz
RUN tar xzf protobuf-cpp-3.1.0.tar.gz
WORKDIR /root/protobuf-3.1.0
RUN ./configure --prefix=/usr
RUN make
RUN make install

WORKDIR /root
RUN wget https://github.com/google/protobuf/releases/download/v3.1.0/protoc-3.1.0-linux-x86_64.zip
RUN unzip protoc-3.1.0-linux-x86_64.zip -d protoc3
RUN mv protoc3/bin/protoc /usr/bin/

#set up node v8.0.0
RUN wget https://github.com/tj/n/archive/v2.1.0.tar.gz
RUN tar xzf v2.1.0.tar.gz
WORKDIR /root/n-2.1.0
RUN make install
RUN n 8.0.0

WORKDIR /root
# Install required python packages.
# Please make sure you are building the image from Apollo root, or else the file cannot be located.
COPY ./modules/tools/py27_requirements.txt /tmp/
RUN pip install -r /tmp/py27_requirements.txt

# Remove all temporary files.
RUN rm -fr /tmp/*

ENV ROSCONSOLE_FORMAT '${file}:${line} ${function}() [${severity}] [${time}]: ${message}'
ENV PYTHONPATH /apollo/bazel-genfiles:/apollo/third_party/ros/lib/python2.7/dist-packages

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
