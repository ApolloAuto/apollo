FROM apolloauto/apollo:dev-18.04-x86_64-20191111_1530
ENV DEBIAN_FRONTEND=noninteractive

COPY installers /tmp/installers
COPY archive    /tmp/archive

RUN sudo bash /tmp/installers/install_qt.sh
RUN sudo bash /tmp/installers/install_user.sh

# Remove system provided qt symlinks
RUN sudo mv /tmp/archive/ad_rss_lib-1.1.0.tar.gz /home/tmp/ && \
    rm -rf /tmp/archive && \
    rm -rf /tmp/installers

WORKDIR /apollo
USER apollo
