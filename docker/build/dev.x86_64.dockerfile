ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ARG GEOLOC
ARG INSTALL_MODE

COPY installers /tmp/installers

RUN bash /tmp/installers/install_geo_adjustment.sh ${GEOLOC}

RUN bash /tmp/installers/install_modules_base.sh
RUN bash /tmp/installers/install_gpu_support.sh # ${WORKHORSE}

RUN bash /tmp/installers/install_ordinary_modules.sh ${INSTALL_MODE}

RUN bash /tmp/installers/install_drivers_deps.sh ${INSTALL_MODE}
RUN bash /tmp/installers/install_perception_deps.sh ${INSTALL_MODE}
RUN bash /tmp/installers/install_dreamview_deps.sh ${GEOLOC}

RUN bash /tmp/installers/install_contrib_deps.sh ${INSTALL_MODE}
RUN bash /tmp/installers/install_3rdparty_pept_deps.sh ${INSTALL_MODE}

RUN bash /tmp/installers/install_release_deps.sh

RUN bash /tmp/installers/post_install.sh dev
