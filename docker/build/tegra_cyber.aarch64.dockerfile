FROM nvcr.io/nvidia/l4t-base:r32.4.2
LABEL version="1.2"

ENV DEBIAN_FRONTEND=noninteractive
COPY installers/install_nvidia_ml_for_jetson.sh /tmp/installers/

RUN bash /tmp/installers/install_nvidia_ml_for_jetson.sh
# WORKDIR /apollo
