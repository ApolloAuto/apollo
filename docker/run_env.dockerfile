FROM ubuntu:14.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    curl \
    git \
    libatlas-base-dev \
    libboost-all-dev \
    libconsole-bridge-dev \
    libcurl4-openssl-dev \
    libflann-dev \
    libfreetype6-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libhdf5-serial-dev \
    libicu-dev \
    libleveldb-dev \
    liblmdb-dev \
    liblog4cxx10 \
    liblz4-dev \
    libopencv-dev \
    libopenni-dev \
    libpoco-dev \
    libproj-dev \
    libpython2.7-dev \
    libqhull-dev \
    libsnappy-dev \
    libtinyxml-dev \
    libvtk5-dev \
    libvtk5-qt4-dev \
    libyaml-cpp-dev \
    libyaml-dev \
    mpi-default-dev \
    python-matplotlib \
    python-pip \
    python-scipy \
    python-software-properties \
    realpath \
    software-properties-common \
    tmux \
    unzip \
    wget

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

ENV ROSCONSOLE_FORMAT '${file}:${line} ${function}() [${severity}] [${time}]: ${message}'

# set up node v8.0.0
WORKDIR /tmp
RUN wget https://github.com/tj/n/archive/v2.1.0.tar.gz
RUN tar xzf v2.1.0.tar.gz
WORKDIR /tmp/n-2.1.0
RUN make install
RUN n 8.0.0

WORKDIR /tmp
# Install required python packages.
RUN pip install -r py27_requirements.txt

# https://stackoverflow.com/questions/25193161/chfn-pam-system-error-intermittently-in-docker-hub-builds
RUN ln -s -f /bin/true /usr/bin/chfn

# install Opengl
RUN echo "deb http://ppa.launchpad.net/keithw/glfw3/ubuntu trusty main" | sudo tee -a /etc/apt/sources.list.d/fillwave_ext.list
RUN echo "deb-src http://ppa.launchpad.net/keithw/glfw3/ubuntu trusty main" | sudo tee -a /etc/apt/sources.list.d/fillwave_ext.list
RUN apt-get update && apt-get install -y --force-yes libglfw3 libglfw3-dev

WORKDIR /tmp
RUN wget https://github.com/nigels-com/glew/releases/download/glew-2.0.0/glew-2.0.0.zip
RUN unzip glew-2.0.0.zip
WORKDIR /tmp/glew-2.0.0
RUN make && make install
RUN ln -s /usr/lib64/libGLEW.so /usr/lib/libGLEW.so
RUN ln -s /usr/lib64/libGLEW.so.2.0 /usr/lib/libGLEW.so.2.0

# Remove all temporary files.
RUN rm -fr /tmp/*
