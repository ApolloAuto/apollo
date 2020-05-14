FROM apolloauto/apollo:cyber-x86_64-18.04-20200510_2227
ARG GEOLOC
ARG BUILD_STAGE

WORKDIR /apollo
USER root

COPY installers /tmp/installers
RUN bash /tmp/installers/install_geo_adjustment.sh ${GEOLOC}

COPY archive /tmp/archive

# Better put tensorrt before caffe
RUN bash /tmp/installers/install_tensorrt.sh
RUN bash /tmp/installers/install_gpu_caffe.sh

RUN bash /tmp/installers/install_map_deps.sh
RUN bash /tmp/installers/install_prediction_deps.sh
RUN bash /tmp/installers/install_proj4.sh
RUN bash /tmp/installers/install_osqp.sh
RUN bash /tmp/installers/install_qp_oases.sh
RUN bash /tmp/installers/install_libtorch.sh
RUN bash /tmp/installers/post_install.sh ${BUILD_STAGE}
