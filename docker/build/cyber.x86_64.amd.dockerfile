ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG APOLLO_DIST
ARG GEOLOC
ARG CLEAN_DEPS
ARG INSTALL_MODE

LABEL version="1.2"

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH /opt/apollo/sysroot/bin:$PATH
ENV APOLLO_DIST ${APOLLO_DIST}

COPY installers /opt/apollo/installers
COPY rcfiles /opt/apollo/rcfiles

RUN bash /opt/apollo/installers/install_minimal_environment.sh ${GEOLOC}
RUN bash /opt/apollo/installers/install_cmake.sh
RUN bash /opt/apollo/installers/install_cyber_deps.sh ${INSTALL_MODE}
RUN bash /opt/apollo/installers/install_llvm_clang.sh
RUN bash /opt/apollo/installers/install_qa_tools.sh
RUN bash /opt/apollo/installers/install_visualizer_deps.sh
RUN bash /opt/apollo/installers/install_bazel.sh
RUN bash /opt/apollo/installers/post_install.sh cyber

WORKDIR /apollo
