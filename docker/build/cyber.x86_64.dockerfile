ARG BASE_IMAGE=nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04
# ARG BASE_IMAGE=ubuntu:18.04
FROM ${BASE_IMAGE}

LABEL version="1.0"

ENV DEBIAN_FRONTEND=noninteractive

RUN apt clean
RUN apt update -y && \
    apt install -y \
    build-essential \
    gcc-4.8 \
    g++-4.8 \
    cmake \
    curl \
    git \
    unzip \
    vim \
    wget \
    bc \
    gdb \
    uuid-dev \
    python \
    python-dev \
    python3 \
    python3-dev \
    qt5-default \
    libasio-dev \
    libtinyxml2-6 \
    libtinyxml2-dev \
    libncurses5-dev \
    libavcodec57 \
    libavcodec-dev \
    libconsole-bridge-dev \
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
    software-properties-common

#install gcc 4.8.5
RUN rm -f /usr/bin/gcc
RUN ln -s /usr/bin/gcc-4.8 /etc/alternatives/gcc
RUN ln -s /etc/alternatives/gcc /usr/bin/gcc
RUN rm -f /usr/bin/g++
RUN ln -s /usr/bin/g++-4.8 /etc/alternatives/g++
RUN ln -s /etc/alternatives/g++ /usr/bin/g++

# Run installer
COPY installers /tmp/installers
RUN bash /tmp/installers/install_bazel.sh
RUN bash /tmp/installers/install_gflags_glog.sh
RUN bash /tmp/installers/install_protobuf.sh
RUN bash /tmp/installers/install_bazel_packages.sh
RUN bash /tmp/installers/install_google_styleguide.sh
RUN bash /tmp/installers/install_osqp.sh
RUN bash /tmp/installers/install_python_modules.sh

# Add Bionic source
RUN echo "deb http://us.archive.ubuntu.com/ubuntu/ bionic main restricted" > /etc/apt/sources.list
RUN echo "deb http://us.archive.ubuntu.com/ubuntu/ bionic-updates main restricted" >> /etc/apt/sources.list
RUN echo "deb http://us.archive.ubuntu.com/ubuntu/ bionic universe" >> /etc/apt/sources.list
RUN echo "deb http://us.archive.ubuntu.com/ubuntu/ bionic-updates universe" >> /etc/apt/sources.list
RUN echo "deb http://us.archive.ubuntu.com/ubuntu/ bionic multiverse" >> /etc/apt/sources.list
RUN echo "deb http://us.archive.ubuntu.com/ubuntu/ bionic-updates multiverse" >> /etc/apt/sources.list
RUN echo "deb http://us.archive.ubuntu.com/ubuntu/ bionic-backports main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo "deb http://security.ubuntu.com/ubuntu bionic-security main restricted" >> /etc/apt/sources.list
RUN echo "deb http://security.ubuntu.com/ubuntu bionic-security multiverse" >> /etc/apt/sources.list

#add Trusty universe into apt source for Poco foundation 9
RUN echo "deb http://dk.archive.ubuntu.com/ubuntu/ trusty main" >> /etc/apt/sources.list
RUN echo "deb http://dk.archive.ubuntu.com/ubuntu/ trusty universe" >> /etc/apt/sources.list
RUN apt update -y
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
    libgdal-dev \
    libvtk6-dev \
    libvtk6.3 \
    vtk6 \
    libpocofoundation9
RUN rm -f /usr/lib/libPocoFoundation.so
RUN ln -s /usr/lib/libPocoFoundation.so.9 /usr/lib/libPocoFoundation.so
RUN ln -s /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.54.0 /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
RUN ln -s /usr/lib/x86_64-linux-gnu/libboost_iostreams.so.1.54.0 /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
RUN ln -s /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.65.1 /usr/lib/x86_64-linux-gnu/libboost_date_time.so
RUN ln -s /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.65.1 /usr/lib/x86_64-linux-gnu/libboost_regex.so
RUN ln -s /usr/lib/x86_64-linux-gnu/libboost_serialization.so.1.65.1 /usr/lib/x86_64-linux-gnu/libboost_serialization.so
RUN ln -s /usr/lib/x86_64-linux-gnu/libboost_signals.so.1.54.0 /usr/lib/x86_64-linux-gnu/libboost_signals.so
RUN ln -s /usr/lib/x86_64-linux-gnu/libboost_system.so.1.54.0 /usr/lib/x86_64-linux-gnu/libboost_system.so
RUN ln -s /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.54.0 /usr/lib/x86_64-linux-gnu/libboost_thread.so
RUN ln -s /usr/lib/x86_64-linux-gnu/libboost_wserialization.so.1.65.1 /usr/lib/x86_64-linux-gnu/libboost_wserialization.so
RUN ln -s /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.54.0 /usr/lib/x86_64-linux-gnu/libboost_chrono.so
RUN ln -s /usr/lib/python2.7/dist-packages/vtk/libvtkRenderingPythonTkWidgets.x86_64-linux-gnu.so /usr/lib/x86_64-linux-gnu/libvtkRenderingPythonTkWidgets.so

RUN bash /tmp/installers/install_fast-rtps.sh
RUN bash /tmp/installers/install_pcl.sh

WORKDIR /apollo
#USER apollo
