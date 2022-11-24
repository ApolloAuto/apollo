ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG GEOLOC
ARG CLEAN_DEPS
ARG APOLLO_DIST
ARG INSTALL_MODE

# Note(storypku):
# Comment this line if nothing in the `installers` dir got changed.
# We can use installers shipped with CyberRT Docker image.
COPY installers /opt/apollo/installers

RUN bash /opt/apollo/installers/install_geo_adjustment.sh ${GEOLOC}

RUN bash /opt/apollo/installers/install_modules_base.sh
RUN bash /opt/apollo/installers/install_ordinary_modules.sh ${INSTALL_MODE}
RUN bash /opt/apollo/installers/install_drivers_deps.sh ${INSTALL_MODE}
RUN bash /opt/apollo/installers/install_dreamview_deps.sh ${GEOLOC}
RUN bash /opt/apollo/installers/install_contrib_deps.sh ${INSTALL_MODE}
RUN bash /opt/apollo/installers/install_gpu_support.sh
RUN bash /opt/apollo/installers/install_release_deps.sh

# RUN bash /opt/apollo/installers/install_geo_adjustment.sh us

RUN bash /opt/apollo/installers/post_install.sh dev

RUN mkdir -p /opt/apollo/neo/data/log && chmod -R 777 /opt/apollo/neo

COPY rcfiles/setup.sh /opt/apollo/neo/   

RUN echo "source /opt/apollo/neo/setup.sh" >> /etc/skel/.bashrc

RUN echo "deb https://apollo-pkg-beta.bj.bcebos.com/neo/beta bionic main" >> /etc/apt/sources.list
