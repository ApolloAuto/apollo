ARG BASE_IMAGE_CPU=apolloauto/apollo:cyber-aarch64-18.04-20200717_0327
ARG BASE_IMAGE_GPU=apolloauto/apollo:cyber-aarch64-18.04-20200719_0434

FROM ${BASE_IMAGE_GPU}

ARG GEOLOC
#ARG WORKHORSE
ARG BUILD_STAGE
ARG INSTALL_MODE

WORKDIR /apollo

COPY archive /tmp/archive
COPY installers /tmp/installers

RUN bash /tmp/installers/install_geo_adjustment.sh ${GEOLOC}
RUN bash /tmp/installers/install_modules_base.sh

RUN bash /tmp/installers/install_gpu_support.sh # ${WORKHORSE}
#RUN bash /tmp/installers/install_common_modules.sh ${INSTALL_MODE}
#RUN bash /tmp/installers/install_drivers_deps.sh ${INSTALL_MODE}
#RUN bash /tmp/installers/install_perception_deps.sh ${INSTALL_MODE}
#RUN bash /tmp/installers/install_dreamview_deps.sh ${GEOLOC}

#RUN bash /tmp/installers/install_contrib_deps.sh ${INSTALL_MODE}
#RUN bash /tmp/installers/install_3rdparty_pept_deps.sh ${INSTALL_MODE}

#RUN bash /tmp/installers/install_release_deps.sh
RUN bash /tmp/installers/post_install.sh ${BUILD_STAGE}
