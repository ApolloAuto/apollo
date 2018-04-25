FROM nvidia/cuda:8.0-cudnn7-devel-ubuntu14.04

ENV DEBIAN_FRONTEND=noninteractive

# Run installers.
COPY installers /tmp/installers
RUN bash /tmp/installers/pre_install.sh
RUN bash /tmp/installers/install_bazel.sh
RUN bash /tmp/installers/install_bazel_packages.sh
RUN bash /tmp/installers/install_ceres_solver.sh
RUN bash /tmp/installers/install_gflags_glog.sh
RUN bash /tmp/installers/install_glew.sh
RUN bash /tmp/installers/install_glusterfs.sh
RUN bash /tmp/installers/install_gpu_caffe.sh
RUN bash /tmp/installers/install_ipopt.sh
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
RUN bash /tmp/installers/install_yarn.sh
RUN bash /tmp/installers/post_install.sh

RUN apt-get install -y \
   bc \
   cppcheck \
   cuda \
   debconf-utils \
   doxygen \
   graphviz \
   gdb \
   git \
   subversion \
   google-perftools \
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
