FROM apolloauto/apollo:cyber-x86_64-18.04-20200714_0029

ARG GEOLOC
ARG BUILD_STAGE
ARG INSTALL_MODE

WORKDIR /apollo

COPY archive /tmp/archive
COPY installers /tmp/installers

RUN bash /tmp/installers/install_geo_adjustment.sh ${GEOLOC}

RUN bash /tmp/installers/install_modules_base.sh
RUN bash /tmp/installers/install_gpu_support.sh ${INSTALL_MODE}

RUN bash /tmp/installers/install_common_modules.sh ${INSTALL_MODE}

RUN bash /tmp/installers/install_drivers_deps.sh ${INSTALL_MODE}
RUN bash /tmp/installers/install_perception_deps.sh ${INSTALL_MODE}
RUN bash /tmp/installers/install_dreamview_deps.sh ${GEOLOC}

RUN bash /tmp/installers/install_contrib_deps.sh ${INSTALL_MODE}
RUN bash /tmp/installers/install_3rdparty_pept_deps.sh ${INSTALL_MODE}

RUN bash /tmp/installers/install_release_stage.sh

RUN bash /tmp/installers/post_install.sh ${BUILD_STAGE}
