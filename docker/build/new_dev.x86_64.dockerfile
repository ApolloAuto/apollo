FROM apolloauto/apollo:cyber-x86_64-18.04-20200510_2227
ARG GEOLOC
ARG BUILD_STAGE

#TODO(ALL): remove this!
WORKDIR /apollo
USER root

COPY installers /tmp/installers
RUN bash /tmp/installers/install_us2cn_adjustment.sh ${GEOLOC}


RUN apt-get -y update && \
    apt-get -y install \
    silversearcher-ag && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY archive /tmp/archive
RUN bash /tmp/installers/install_tensorrt.sh
RUN bash /tmp/installers/post_install.sh ${BUILD_STAGE}


