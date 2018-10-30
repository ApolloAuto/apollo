#FROM nvidia/cuda:8.0-cudnn7-devel-ubuntu14.04
FROM arm64v8/ubuntu:14.04

ENV DEBIAN_FRONTEND=noninteractive

# Run installers.
COPY installers /tmp/installers
RUN bash /tmp/installers/pre_install.sh
RUN bash /tmp/installers/install_adv_plat.sh
RUN bash /tmp/installers/install_bazel_aarch64.sh
RUN bash /tmp/installers/install_bazel_packages.sh
#RUN bash /tmp/installers/install_conda.sh
RUN bash /tmp/installers/install_gflags_glog_aarch64.sh
RUN bash /tmp/installers/install_glew_aarch64.sh
RUN bash /tmp/installers/install_glusterfs.sh
RUN bash /tmp/installers/install_gpu_caffe_aarch64.sh
RUN bash /tmp/installers/install_ipopt_aarch64.sh
RUN bash /tmp/installers/install_libjsonrpc-cpp.sh
RUN bash /tmp/installers/install_nlopt.sh
RUN bash /tmp/installers/install_node_aarch64.sh
RUN bash /tmp/installers/install_ota.sh
RUN bash /tmp/installers/install_pcl_aarch64.sh
RUN bash /tmp/installers/install_protobuf.sh
RUN bash /tmp/installers/install_python_modules.sh
RUN bash /tmp/installers/install_qp_oases.sh
RUN bash /tmp/installers/install_ros_aarch64.sh
RUN bash /tmp/installers/install_snowboy_aarch64.sh
RUN bash /tmp/installers/install_supervisor.sh
RUN bash /tmp/installers/install_undistort.sh
RUN bash /tmp/installers/install_user.sh
RUN bash /tmp/installers/install_yarn.sh
RUN bash /tmp/installers/install_cuda_aarch64.sh
RUN bash /tmp/installers/post_install.sh
RUN bash /tmp/installers/install_google_perftools_aarch64.sh

RUN apt-get install -y \
   bc \
   cppcheck \
   debconf-utils \
   doxygen \
   graphviz \
   gdb \
   git \
   subversion \
   lcov \
   libblas-dev \
   libboost-all-dev \
   libcurl4-openssl-dev \
   libfreetype6-dev \
   liblapack-dev \
   libpcap-dev \
   locate \
   lsof \
   realpath \
   shellcheck \
   vim \
   v4l-utils \
   nfs-common \
   zip

WORKDIR /apollo
USER apollo
