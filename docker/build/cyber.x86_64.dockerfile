ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG APOLLO_DIST
ARG GEOLOC

LABEL version="1.2"

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH /opt/apollo/sysroot/bin:$PATH
ENV APOLLO_DIST ${APOLLO_DIST}

COPY installers /tmp/installers
COPY rcfiles /opt/apollo/rcfiles

# (Deprecated) Pre-downloaded tarballs
# COPY archive /tmp/archive

RUN bash /tmp/installers/install_minimal_environment.sh ${GEOLOC}
RUN bash /tmp/installers/install_cmake.sh
RUN bash /tmp/installers/install_cyber_deps.sh
RUN bash /tmp/installers/install_llvm_clang.sh
RUN bash /tmp/installers/install_qa_tools.sh
RUN bash /tmp/installers/install_visualizer_deps.sh
RUN bash /tmp/installers/install_bazel.sh
RUN bash /tmp/installers/post_install.sh cyber

WORKDIR /apollo
