FROM apolloauto/apollo:dev-18.04-x86_64-20200428_2300
ENV DEBIAN_FRONTEND=noninteractive

COPY installers /tmp/installers
RUN sudo bash /tmp/installers/install_tensorrt_temp.sh

WORKDIR /apollo
USER apollo
