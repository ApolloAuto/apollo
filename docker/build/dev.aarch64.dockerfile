ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG GEOLOC
ARG INSTALL_MODE

WORKDIR /apollo

COPY archive /tmp/archive

COPY installers /tmp/installers

RUN bash /tmp/installers/install_geo_adjustment.sh ${GEOLOC}
RUN bash /tmp/installers/install_modules_base.sh
RUN bash /tmp/installers/install_gpu_support.sh
RUN bash /tmp/installers/install_ordinary_modules.sh
RUN bash /tmp/installers/install_drivers_deps.sh ${INSTALL_MODE}
RUN bash /tmp/installers/install_dreamview_deps.sh ${GEOLOC}

RUN bash /tmp/installers/install_contrib_deps.sh

RUN bash /tmp/installers/install_release_deps.sh
RUN bash /tmp/installers/post_install.sh ${BUILD_STAGE}
