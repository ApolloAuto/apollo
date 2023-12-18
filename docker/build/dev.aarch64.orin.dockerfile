FROM nvcr.io/nvidia/l4t-jetpack:r35.2.1 
ENV CUDA_LITE 11.4

RUN apt-get update && apt-get install -y --no-install-recommends sudo gnupg2 curl ca-certificates \
    && rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 11.4.1

#RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/sbsa/cuda-ubuntu2004.pin && \
#    sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
#    sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/sbsa/7fa2af80.pub \
#    && sudo add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/sbsa/ /" && \
#    sudo apt-get update


ENV PATH /usr/local/cuda/bin:${PATH}

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=${CUDA_LITE}"


ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs

ENV CUDNN_VERSION 8.6.0.1665

ENV TENSORRT_VERSION 8.5.2
 
COPY rcfiles /opt/apollo/rcfiles
COPY installers /opt/apollo/installers

RUN apt update && DEBIAN_FRONTEND=noninteractive TZ="Asian/China" apt-get -y install tzdata

RUN bash /opt/apollo/installers/install_minimal_environment.sh cn 20.04
RUN bash /opt/apollo/installers/install_bazel.sh
RUN bash /opt/apollo/installers/install_cmake.sh build

RUN bash /opt/apollo/installers/install_llvm_clang.sh
RUN bash /opt/apollo/installers/install_cyber_deps.sh build
RUN bash /opt/apollo/installers/install_qa_tools.sh
RUN bash /opt/apollo/installers/install_visualizer_deps.sh build 20.04

RUN bash /opt/apollo/installers/install_geo_adjustment.sh us

RUN bash /opt/apollo/installers/install_modules_base.sh
RUN bash /opt/apollo/installers/install_ordinary_modules.sh build
RUN bash /opt/apollo/installers/install_drivers_deps.sh build
RUN bash /opt/apollo/installers/install_dreamview_deps.sh cn
RUN bash /opt/apollo/installers/install_contrib_deps.sh build
RUN bash /opt/apollo/installers/install_gpu_support.sh
RUN bash /opt/apollo/installers/install_release_deps.sh
RUN bash /opt/apollo/installers/install_tkinter.sh

# RUN bash /opt/apollo/installers/install_geo_adjustment.sh us

RUN bash /opt/apollo/installers/post_install.sh dev

RUN mkdir -p /opt/apollo/neo/data/log && chmod -R 777 /opt/apollo/neo

COPY rcfiles/setup.sh /opt/apollo/neo/   

RUN echo "source /opt/apollo/neo/setup.sh" >> /etc/skel/.bashrc

RUN wget "https://apollo-system.cdn.bcebos.com/patch/libc-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb" \
    && wget "https://apollo-system.cdn.bcebos.com/patch/libc-dev-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb" \
    && wget "https://apollo-system.cdn.bcebos.com/patch/libc6-dev_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb" \
    && wget "https://apollo-system.cdn.bcebos.com/patch/libc6_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb" \
    && dpkg -i libc-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb libc-dev-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb libc6-dev_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb libc6_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb \
    && rm -f libc-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb libc-dev-bin_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb libc6-dev_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb libc6_2.31-0ubuntu9.9.ubuntu.focal.custom_arm64.deb 

RUN sed -i 's/#include "flann\/general\.h"/#include <\/usr\/include\/flann\/general\.h>/g' /usr/include/flann/util/params.h
