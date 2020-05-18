FROM apolloauto/apollo:cyber-x86_64-18.04-20200514_1653

ARG GEOLOC
ARG BUILD_STAGE
ARG INSTALL_MODE

WORKDIR /apollo
USER root

COPY installers /tmp/installers
RUN bash /tmp/installers/install_geo_adjustment.sh ${GEOLOC}

COPY archive /tmp/archive

# Better put tensorrt before caffe
RUN bash /tmp/installers/install_tensorrt.sh
RUN bash /tmp/installers/install_gpu_caffe.sh ${INSTALL_MODE}

RUN bash /tmp/installers/install_drivers_deps.sh

RUN bash /tmp/installers/install_map_deps.sh
RUN bash /tmp/installers/install_prediction_deps.sh
RUN bash /tmp/installers/install_adolc.sh
RUN bash /tmp/installers/install_ffmpeg.sh
RUN bash /tmp/installers/install_proj4.sh
RUN bash /tmp/installers/install_tf2.sh
RUN bash /tmp/installers/install_osqp.sh
RUN bash /tmp/installers/install_qp_oases.sh
RUN bash /tmp/installers/install_libtorch.sh
RUN bash /tmp/installers/post_install.sh ${BUILD_STAGE}
