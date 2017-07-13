FROM ubuntu:14.04

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
    python-matplotlib \
    python-scipy \
    python-software-properties \
    realpath \
    tmux \
    unzip \
    wget  \
    libtinyxml-dev \
    libpoco-dev \
    libproj-dev \
    liblz4-dev \
    libconsole-bridge-dev \
    git

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

ENV ROSCONSOLE_FORMAT '${file}:${line} ${function}() [${severity}] [${time}]: ${message}'

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

# Remove all temporary files.
RUN rm -fr /tmp/*

# https://stackoverflow.com/questions/25193161/chfn-pam-system-error-intermittently-in-docker-hub-builds
RUN ln -s -f /bin/true /usr/bin/chfn
