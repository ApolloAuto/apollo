package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "perception_proto",
    deps = [
        ":perception_proto_lib",
    ],
)

proto_library(
    name = "perception_proto_lib",
    srcs = [
        "perception_obstacle.proto",
        "traffic_light_detection.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/map/proto:map_proto_lib",
    ],
)

cc_proto_library(
    name = "lane_post_process_config_lib_proto",
    deps = [
        ":lane_post_process_config_lib",
    ],
)

proto_library(
    name = "lane_post_process_config_lib",
    srcs = [
        "lane_post_process_config.proto",
    ],
)

cc_proto_library(
    name = "probabilistic_fusion_config_lib_proto",
    deps = [
        ":probabilistic_fusion_config_lib",
    ],
)

proto_library(
    name = "probabilistic_fusion_config_lib",
    srcs = [
        "probabilistic_fusion_config.proto",
    ],
)

cc_proto_library(
    name = "yolo_camera_detector_config_lib_proto",
    deps = [
        ":yolo_camera_detector_config_lib",
    ],
)

proto_library(
    name = "yolo_camera_detector_config_lib",
    srcs = [
        "yolo_camera_detector_config.proto",
    ],
)

cc_proto_library(
    name = "modest_radar_detector_config_lib_proto",
    deps = [
        ":modest_radar_detector_config_lib",
    ],
)

proto_library(
    name = "modest_radar_detector_config_lib",
    srcs = [
        "modest_radar_detector_config.proto",
    ],
)
