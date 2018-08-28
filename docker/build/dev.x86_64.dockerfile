FROM nvidia/cuda:8.0-cudnn7-devel-ubuntu14.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && \
    apt-get install -y \
    apt-transport-https \
    bc \
    build-essential \
    cmake \
    cppcheck \
    curl \
    debconf-utils \
    doxygen \
    gdb \
    git \
    google-perftools \
    graphviz \
    lcov \
    libblas-dev \
    libboost-all-dev \
    libcurl4-openssl-dev \
    libfreetype6-dev \
    liblapack-dev \
    libpcap-dev \
    locate \
    lsof \
    nfs-common \
    realpath \
    shellcheck \
    software-properties-common \
    sshfs \
    subversion \
    unzip \
    v4l-utils \
    vim \
    wget \
    zip && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    echo '\n\n\n' | ssh-keygen -t rsa

# Run installers.
COPY installers /tmp/installers
RUN bash /tmp/installers/install_adv_plat.sh
RUN bash /tmp/installers/install_bazel.sh
RUN bash /tmp/installers/install_bazel_packages.sh
RUN bash /tmp/installers/install_conda.sh
RUN bash /tmp/installers/install_gflags_glog.sh
RUN bash /tmp/installers/install_glew.sh
RUN bash /tmp/installers/install_gpu_caffe.sh
RUN bash /tmp/installers/install_ipopt.sh
RUN bash /tmp/installers/install_libjsonrpc-cpp.sh
RUN bash /tmp/installers/install_nlopt.sh
RUN bash /tmp/installers/install_node.sh
RUN bash /tmp/installers/install_ota.sh
RUN bash /tmp/installers/install_pcl.sh
RUN bash /tmp/installers/install_protobuf.sh
RUN bash /tmp/installers/install_python_modules.sh
RUN bash /tmp/installers/install_qp_oases.sh
RUN bash /tmp/installers/install_ros.sh
RUN bash /tmp/installers/install_snowboy.sh
RUN bash /tmp/installers/install_supervisor.sh
RUN bash /tmp/installers/install_undistort.sh
RUN bash /tmp/installers/install_user.sh
RUN bash /tmp/installers/install_yarn.sh
RUN bash /tmp/installers/post_install.sh

WORKDIR /apollo
USER apollo
