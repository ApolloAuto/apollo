FROM ubuntu:14.04

WORKDIR /root

RUN apt-get update && apt-get install -y \
    curl \
    build-essential \
    libboost-all-dev \
    libcurl4-openssl-dev \
    libfreetype6-dev \
    liblog4cxx10 \
    libpython2.7-dev \
    libyaml-cpp-dev \
    libyaml-dev \
    python-pip \
    realpath \
    tmux \
    unzip \
    wget

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

ENV ROSCONSOLE_FORMAT '${file}:${line} ${function}() [${severity}] [${time}]: ${message}'
ENV PYTHONPATH /apollo/lib:/apollo/ros/lib/python2.7/dist-packages

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
