FROM nvcr.io/nvidia/l4t-base:r32.4.2

ARG BUILD_STAGE
ARG GEOLOC
ARG INSTALL_MODE

LABEL version="1.2"

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH /opt/apollo/sysroot/bin:$PATH

COPY installers /tmp/installers
COPY rcfiles /opt/apollo/rcfiles
# Pre-downloaded tarballs
COPY archive /tmp/archive

RUN bash /tmp/installers/install_minimal_environment.sh ${GEOLOC}
RUN bash /tmp/installers/install_bazel.sh
RUN bash /tmp/installers/install_cmake.sh ${INSTALL_MODE}
RUN bash /tmp/installers/install_llvm_clang.sh

RUN bash /tmp/installers/install_cyber_deps.sh
RUN bash /tmp/installers/install_qa_tools.sh

RUN bash /tmp/installers/install_visualizer_deps.sh
RUN bash /tmp/installers/post_install.sh ${BUILD_STAGE}

WORKDIR /apollo
