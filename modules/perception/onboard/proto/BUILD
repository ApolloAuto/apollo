load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "radar_component_config_proto",
    deps = [
        ":radar_component_config_proto_lib",
    ],
)

proto_library(
    name = "radar_component_config_proto_lib",
    srcs = ["radar_component_config.proto"],
)

cc_proto_library(
    name = "fusion_component_config_proto",
    deps = [
        ":fusion_component_config_proto_lib",
    ],
)

proto_library(
    name = "fusion_component_config_proto_lib",
    srcs = ["fusion_component_config.proto"],
)

cc_proto_library(
    name = "fusion_camera_detection_proto",
    deps = [
        ":fusion_camera_detection_proto_lib",
    ],
)

proto_library(
    name = "fusion_camera_detection_proto_lib",
    srcs = ["fusion_camera_detection_component.proto"],
)

cc_proto_library(
    name = "trafficlights_perception_component_proto",
    deps = [
        ":trafficlights_perception_component_proto_lib",
    ],
)

proto_library(
    name = "trafficlights_perception_component_proto_lib",
    srcs = ["trafficlights_perception_component.proto"],
)

cc_proto_library(
    name = "lane_perception_component_proto",
    deps = [
        ":lane_perception_component_proto_lib",
    ],
)

proto_library(
    name = "lane_perception_component_proto_lib",
    srcs = ["lane_perception_component.proto"],
)

cc_proto_library(
    name = "motion_service_proto",
    deps = [
        ":motion_service_proto_lib",
    ],
)

proto_library(
    name = "motion_service_proto_lib",
    srcs = ["motion_service.proto"],
)

cc_proto_library(
    name = "lidar_component_config_proto",
    deps = [
        ":lidar_component_config_proto_lib",
    ],
)

proto_library(
    name = "lidar_component_config_proto_lib",
    srcs = ["lidar_component_config.proto"],
)

cpplint()
