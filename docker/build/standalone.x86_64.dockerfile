ARG DOCKER_REPO=apolloauto/apollo
ARG TARGET_ARCH=x86_64
ARG IMAGE_VERSION=18.04-20210428_1718
ARG BASE_IMAGE=${DOCKER_REPO}:runtime-${TARGET_ARCH}-${IMAGE_VERSION}

ARG DOCKER_USER
ARG DOCKER_USER_ID
ARG DOCKER_GRP
ARG DOCKER_GRP_ID

FROM ${DOCKER_REPO}:data_volume-audio_model-${TARGET_ARCH}-latest as apollo_audio_volume
FROM ${DOCKER_REPO}:yolov4_volume-emergency_detection_model-${TARGET_ARCH}-latest as apollo_yolov4_volume
FROM ${DOCKER_REPO}:faster_rcnn_volume-traffic_light_detection_model-${TARGET_ARCH}-latest as apollo_faster_rcnn_volume
FROM ${DOCKER_REPO}:smoke_volume-yolo_obstacle_detection_model-${TARGET_ARCH}-latest as apollo_smoke_volume

FROM ${BASE_IMAGE}

ENV DOCKER_USER=${DOCKER_USER:-apollo}
ENV DOCKER_USER_ID=${DOCKER_USER_ID:-1001}
ENV DOCKER_GRP=${DOCKER_GRP:-apollo}
ENV DOCKER_GRP_ID=${DOCKER_GRP_ID:-1001}

# We need to copy output first to make sure that the top-level /apollo directory is also owned by ${DOCKER_USER_ID}:${DOCKER_GRP_ID},
# because COPY --chown creates target directory as root and only chowns the files it copies.
# If we copy some volume first then /apollo is owned by root and different user will fail to create e.g.
# nohup.out or /apollo/data there (in runtime image it's overlayed by a volume from host, so it doesn't harm)
COPY \
    --chown=${DOCKER_USER_ID}:${DOCKER_GRP_ID} \
    output \
    /apollo

COPY \
    --from=apollo_audio_volume \
    --chown=${DOCKER_USER_ID}:${DOCKER_GRP_ID} \
    /apollo/modules/audio \
    /apollo/modules/audio

COPY \
    --from=apollo_yolov4_volume \
    --chown=${DOCKER_USER_ID}:${DOCKER_GRP_ID} \
    /apollo/modules/perception/camera/lib/obstacle/detector/yolov4 \
    /apollo/modules/perception/camera/lib/obstacle/detector/yolov4

COPY \
    --from=apollo_faster_rcnn_volume \
    --chown=${DOCKER_USER_ID}:${DOCKER_GRP_ID} \
    /apollo/modules/perception/production/data/perception/camera/models/traffic_light_detection \
    /apollo/modules/perception/production/data/perception/camera/models/traffic_light_detection

COPY \
    --from=apollo_smoke_volume \
    --chown=${DOCKER_USER_ID}:${DOCKER_GRP_ID} \
    /apollo/modules/perception/production/data/perception/camera/models/yolo_obstacle_detector \
    /apollo/modules/perception/production/data/perception/camera/models/yolo_obstacle_detector

RUN /apollo/scripts/docker_start_user.sh
