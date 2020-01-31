FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

# Common tools.
RUN apt update -y && \
    apt install -y \
        bc \
        cmake \
        g++-8 \
        git \
        python3-pip \
        sudo \
        unzip \
        wget \
        zip && \
    apt clean

# Install in-system libs which will be imported into bazel WORKSPACE by
# third_party/BUILD.
RUN apt update -y && \
    apt install -y \
        coinor-libipopt-dev \
        libadolc-dev \
        libavcodec-dev \
        libavutil-dev \
        libboost-dev \
        libcaffe-cuda-dev \
        libncurses5-dev \
        libomp-dev \
        libopencv-dev \
        libpcl-dev \
        libpoco-dev \
        libsqlite3-dev \
        libswresample-dev \
        python3-dev \
        uuid-dev && \
    apt clean

# Make alias for easier compilation.
RUN ln -s /usr/include/pcl-1.8/pcl /usr/include/pcl

# Run customized installers.
COPY installers /tmp/installers
RUN bash /tmp/installers/install_bazel.sh
RUN bash /tmp/installers/install_bazel_packages.sh
RUN bash /tmp/installers/install_bosfs.sh
RUN bash /tmp/installers/install_conda.sh
RUN bash /tmp/installers/install_fastrtps.sh

WORKDIR /apollo
