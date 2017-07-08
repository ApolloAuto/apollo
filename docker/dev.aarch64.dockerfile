FROM aarch64/ubuntu:16.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update
## the following commands are OK, skip running again
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

RUN add-apt-repository ppa:webupd8team/java
RUN apt-get update
RUN echo "oracle-java8-installer shared/accepted-oracle-license-v1-1 select true" | debconf-set-selections
RUN apt-get install -y oracle-java8-installer
RUN apt-get clean autoclean && apt-get autoremove -y
RUN rm -fr /var/lib/apt/lists/*
COPY bazel /usr/local/bin/bazel
RUN mkdir -p /usr/local/bin
WORKDIR /usr/local/bin/
RUN wget https://github.com/startcode/bazel-arm64/releases/download/0.4.4/bazel-aarch64 && ln -rs bazel-aarch64 bazel

WORKDIR /tmp
## install protobuf 3.1.0
RUN wget https://github.com/google/protobuf/releases/download/v3.1.0/protobuf-cpp-3.1.0.tar.gz
RUN tar xzf protobuf-cpp-3.1.0.tar.gz
WORKDIR /tmp/protobuf-3.1.0
RUN ./configure --prefix=/usr
RUN make
RUN make install

# set up node v8.0.0
RUN curl -sL https://deb.nodesource.com/setup_8.x | bash -
RUN apt-get install -y nodejs

## Install required python packages.
WORKDIR /tmp
COPY ./modules/tools/py27_requirements.txt /tmp/
RUN pip install -r py27_requirements.txt

# Install yarn
RUN curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | apt-key add -
RUN echo "deb https://dl.yarnpkg.com/debian/ stable main" | tee /etc/apt/sources.list.d/yarn.list
RUN apt-get update && apt-get install -y yarn

# Remove all temporary files.
RUN rm -fr /tmp/*

ENV ROSCONSOLE_FORMAT '${file}:${line} ${function}() [${severity}] [${time}]: ${message}'

# install dependency for ros build
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
