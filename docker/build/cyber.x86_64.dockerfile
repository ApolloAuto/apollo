FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

# Common tools.
RUN apt update -y && \
    apt install -y \
        bc \
        cmake \
        g++-8 \
        git \
        sudo \
        unzip \
        wget \
        zip && \
    apt clean

# Install in-system libs.
RUN apt update -y && \
    apt install -y \
        libavcodec-dev \
        libavutil-dev \
        libncurses5-dev \
        libpoco-dev \
        libswresample-dev \
        python3-dev \
        uuid-dev && \
    apt clean

# Run customized installers.
COPY installers /tmp/installers
RUN bash /tmp/installers/install_bazel.sh
RUN bash /tmp/installers/install_bazel_packages.sh
RUN bash /tmp/installers/install_fastrtps.sh

WORKDIR /apollo
