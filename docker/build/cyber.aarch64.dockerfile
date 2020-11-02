ARG BASE_IMAGE
# ARG BASE_IMAGE=arm64v8/ubuntu:18.04
FROM ${BASE_IMAGE}

ARG GEOLOC
ARG INSTALL_MODE

LABEL version="1.2"

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH /opt/apollo/sysroot/bin:$PATH

COPY installers /tmp/installers
COPY rcfiles /opt/apollo/rcfiles

RUN bash /tmp/installers/install_minimal_environment.sh ${GEOLOC}
RUN bash /tmp/installers/install_bazel.sh
RUN bash /tmp/installers/install_cmake.sh ${INSTALL_MODE}

RUN bash /tmp/installers/install_llvm_clang.sh
RUN bash /tmp/installers/install_cyber_deps.sh
RUN bash /tmp/installers/install_qa_tools.sh
RUN bash /tmp/installers/install_visualizer_deps.sh ${INSTALL_MODE}

RUN bash /tmp/installers/post_install.sh cyber

WORKDIR /apollo
