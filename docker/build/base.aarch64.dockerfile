ARG BASE_IMAGE
FROM ${BASE_IMAGE}

LABEL version="1.2"

ENV DEBIAN_FRONTEND=noninteractive

ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV PATH /usr/local/cuda/bin:${PATH}

COPY installers/install_jetson_ml.sh /tmp/installers/
RUN bash /tmp/installers/install_jetson_ml.sh \
    && rm -rf /tmp/installers
