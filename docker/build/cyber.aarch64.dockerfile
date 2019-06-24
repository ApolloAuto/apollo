FROM arm64v8/ubuntu:18.04

ARG INSTALL_MODE

LABEL version="1.0"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt clean
RUN apt update -y && \
    apt install -y \
    build-essential \
    cmake \
    curl \
    gcc-5 \
    g++-5 \
    git \
    unzip \
    zip \
    vim \
    wget \
    bc \
    gdb \
    uuid-dev \
    python \
    python-dev \
    python3 \
    python3-dev \
    libasio-dev \
    libtinyxml2-6 \
    libtinyxml2-dev \
    libncurses5-dev \
    libavcodec57 \
    libavcodec-dev \
    libswscale4 \
    libswscale-dev \
    libcurl4-nss-dev \
    libpoco-dev \
    libeigen3-dev \
    libflann-dev \
    libqhull-dev \
    libpcap0.8 \
    libpcap0.8-dev \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    libopenni0 \
    libopenni-dev \
    libopenni2-0 \
    libopenni2-dev \
    openjdk-8-jdk \
    software-properties-common

RUN rm -f /usr/bin/gcc
RUN ln -s /usr/bin/gcc-5 /etc/alternatives/gcc
RUN ln -s /etc/alternatives/gcc /usr/bin/gcc
RUN rm -f /usr/bin/g++
RUN ln -s /usr/bin/g++-5 /etc/alternatives/g++
RUN ln -s /etc/alternatives/g++ /usr/bin/g++

# Run installer [build|download]
COPY installers /tmp/installers
RUN bash /tmp/installers/install_bazel.sh ${INSTALL_MODE}
RUN bash /tmp/installers/install_gflags_glog.sh
RUN bash /tmp/installers/install_protobuf.sh
RUN bash /tmp/installers/install_bazel_packages.sh
RUN bash /tmp/installers/install_google_styleguide.sh
RUN bash /tmp/installers/install_osqp.sh ${INSTALL_MODE}

# Add Bionic source
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports/ bionic main restricted" > /etc/apt/sources.list
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports/ bionic-updates main restricted" >> /etc/apt/sources.list
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports/ bionic universe" >> /etc/apt/sources.list
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports/ bionic-updates universe" >> /etc/apt/sources.list
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports/ bionic multiverse" >> /etc/apt/sources.list
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports/ bionic-updates multiverse" >> /etc/apt/sources.list
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports/ bionic-backports main restricted universe multiverse" >> /etc/apt/sources.list

#add Trusty universe into apt source for Poco foundation 9
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports/ trusty main" >> /etc/apt/sources.list
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports/ trusty universe" >> /etc/apt/sources.list
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports/ xenial main" >> /etc/apt/sources.list
RUN echo "deb http://ports.ubuntu.com/ubuntu-ports/ xenial universe" >> /etc/apt/sources.list
RUN apt update -y
RUN apt remove --purge openmpi-common=2.1.1-8
RUN apt install -y --allow-downgrades \
    libboost-system1.54.0 \
    libboost-thread1.54.0 \
    libboost-signals1.54.0 \
    libboost-filesystem1.54.0 \
    libboost-iostreams1.54.0 \
    libboost-chrono1.54.0 \
    libboost1.54-dev \
    libboost-dev=1.54.0.1ubuntu1 \
    libkml-dev \
    libopencv-core-dev=2.4.8+dfsg1-2ubuntu1 \
    libopencv-imgproc-dev=2.4.8+dfsg1-2ubuntu1 \
    libopencv-highgui-dev=2.4.8+dfsg1-2ubuntu1 \
    openmpi-common=1.10.2-8ubuntu1 \
    openmpi-bin=1.10.2-8ubuntu1 \
    libopenmpi1.10 \
    libopenmpi-dev=1.10.2-8ubuntu1 \
    libgdal-dev \
    libvtk6.2 \
    vtk6=6.2.0+dfsg1-10build1 \
    libvtk6-java=6.2.0+dfsg1-10build1 \
    libxdmf-dev=2.1.dfsg.1-13 \
    mpi-default-dev=1.4 \
    mpi-default-bin=1.4 \
    python-mpi4py=1.3.1+hg20131106-2ubuntu5 \
    python-vtk6=6.2.0+dfsg1-10build1 \
    tcl-vtk6=6.2.0+dfsg1-10build1 \
    libvtk6-dev=6.2.0+dfsg1-10build1 \
    libpocofoundation9v5

RUN rm -f /usr/lib/libPocoFoundation.so
RUN ln -s /usr/lib/libPocoFoundation.so.9 /usr/lib/libPocoFoundation.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_iostreams.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_iostreams.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_date_time.so.1.65.1 /usr/lib/aarch64-linux-gnu/libboost_date_time.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_regex.so.1.65.1 /usr/lib/aarch64-linux-gnu/libboost_regex.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_serialization.so.1.65.1 /usr/lib/aarch64-linux-gnu/libboost_serialization.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_signals.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_signals.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_system.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_system.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_thread.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_thread.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_wserialization.so.1.65.1 /usr/lib/aarch64-linux-gnu/libboost_wserialization.so
RUN ln -s /usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.54.0 /usr/lib/aarch64-linux-gnu/libboost_chrono.so
RUN ln -s /usr/lib/libgl2ps.so.0 /usr/lib/libgl2ps.so
RUN ln -s /usr/lib/python2.7/dist-packages/vtk/libvtkRenderingPythonTkWidgets.aarch64-linux-gnu.so /usr/lib/aarch64-linux-gnu/libvtkRenderingPythonTkWidgets.so

RUN bash /tmp/installers/install_fast-rtps.sh
RUN bash /tmp/installers/install_pcl.sh ${INSTALL_MODE}
RUN rm -fr /tmp/*

WORKDIR /apollo
#USER apollo
