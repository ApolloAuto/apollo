ARG DOCKER_REPO=apolloauto/apollo
ARG TARGET_ARCH=x86_64
ARG IMAGE_VERSION=18.04-20220803_1505
ARG BASE_IMAGE=${DOCKER_REPO}:runtime-${TARGET_ARCH}-${IMAGE_VERSION}

FROM ${DOCKER_REPO}:data_volume-audio_model-${TARGET_ARCH}-latest as apollo_audio_volume
FROM ${DOCKER_REPO}:yolov4_volume-emergency_detection_model-${TARGET_ARCH}-latest as apollo_yolov4_volume
FROM ${DOCKER_REPO}:faster_rcnn_volume-traffic_light_detection_model-${TARGET_ARCH}-latest as apollo_faster_rcnn_volume
FROM ${DOCKER_REPO}:smoke_volume-yolo_obstacle_detection_model-${TARGET_ARCH}-latest as apollo_smoke_volume

FROM ${BASE_IMAGE}

COPY output /apollo

COPY --from=apollo_audio_volume \
    /apollo/modules/audio \
    /apollo/modules/audio

COPY --from=apollo_yolov4_volume \
    /apollo/modules/perception/camera/lib/obstacle/detector/yolov4 \
    /apollo/modules/perception/camera/lib/obstacle/detector/yolov4

COPY --from=apollo_faster_rcnn_volume \
    /apollo/modules/perception/production/data/perception/camera/models/traffic_light_detection \
    /apollo/modules/perception/production/data/perception/camera/models/traffic_light_detection

COPY --from=apollo_smoke_volume \
    /apollo/modules/perception/production/data/perception/camera/models/yolo_obstacle_detector \
    /apollo/modules/perception/production/data/perception/camera/models/yolo_obstacle_detector
