FROM nvidia/cuda:10.0-cudnn7-devel-ubuntu18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update -y && \
    apt-get install -y \
    apt-transport-https \
    autotools-dev \
    automake \
    bc \
    build-essential \
    cmake \
    cppcheck \
    curl \
    curlftpfs \
    debconf-utils \
    doxygen \
    gdb \
    git \
    google-perftools \
    graphviz \
    iproute2 \
    iputils-ping \
    lcov \
    libblas-dev \
    libssl-dev \
    libboost-all-dev \
    libcurl4-openssl-dev \
    libfreetype6-dev \
    liblapack-dev \
    libpcap-dev \
    libsqlite3-dev \
    libgtest-dev \
    locate \
    lsof \
    nfs-common \
    python-autopep8 \
    shellcheck \
    software-properties-common \
    sshfs \
    subversion \
    unzip \
    uuid-dev \
    v4l-utils \
    vim \
    wget \
    libasound2-dev \
    zip && \
    apt-get clean && rm -rf /var/lib/apt/lists/* && \
    echo '\n\n\n' | ssh-keygen -t rsa

# Run installers.
COPY installers /tmp/installers
RUN bash /tmp/installers/install_adv_plat.sh
RUN bash /tmp/installers/install_bazel.sh
RUN bash /tmp/installers/install_bazel_packages.sh
RUN bash /tmp/installers/install_bosfs.sh
RUN bash /tmp/installers/install_conda.sh
RUN bash /tmp/installers/install_ffmpeg.sh
RUN bash /tmp/installers/install_gflags_glog.sh
RUN bash /tmp/installers/install_glew.sh
RUN bash /tmp/installers/install_google_styleguide.sh
RUN bash /tmp/installers/install_gpu_caffe.sh
RUN bash /tmp/installers/install_ipopt.sh
RUN bash /tmp/installers/install_osqp.sh
RUN bash /tmp/installers/install_libjsonrpc-cpp.sh
RUN bash /tmp/installers/install_nlopt.sh
RUN bash /tmp/installers/install_node.sh
RUN bash /tmp/installers/install_openh264.sh
RUN bash /tmp/installers/install_ota.sh
RUN bash /tmp/installers/install_pcl.sh
RUN bash /tmp/installers/install_poco.sh
RUN bash /tmp/installers/install_protobuf.sh
RUN bash /tmp/installers/install_python_modules.sh
RUN bash /tmp/installers/install_qp_oases.sh
RUN bash /tmp/installers/install_qt.sh
RUN bash /tmp/installers/install_supervisor.sh
RUN bash /tmp/installers/install_undistort.sh
RUN bash /tmp/installers/install_user.sh
RUN bash /tmp/installers/install_yarn.sh
RUN bash /tmp/installers/post_install.sh
RUN bash /tmp/installers/install_opuslib.sh

WORKDIR /apollo
USER apollo
