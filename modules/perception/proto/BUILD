load("//tools:cpplint.bzl", "cpplint")

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
        "motion_service.proto",
        "perception_camera.proto",
        "perception_lane.proto",
        "perception_obstacle.proto",
        "traffic_light_detection.proto",
    ],
    deps = [
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/map/proto:map_proto_lib",
    ],
)

cc_proto_library(
    name = "dst_existance_fusion_config_proto",
    deps = [
        ":dst_existance_fusion_config_proto_lib",
    ],
)

proto_library(
    name = "dst_existance_fusion_config_proto_lib",
    srcs = ["dst_existance_fusion_config.proto"],
)

cc_proto_library(
    name = "perception_config_schema_proto",
    deps = [
        ":perception_config_schema_proto_lib",
    ],
)

proto_library(
    name = "perception_config_schema_proto_lib",
    srcs = ["perception_config_schema.proto"],
)

cc_proto_library(
    name = "perception_ultrasonic_proto",
    deps = [
        ":perception_ultrasonic_proto_lib",
    ],
)

proto_library(
    name = "perception_ultrasonic_proto_lib",
    srcs = ["perception_ultrasonic.proto"],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "ccrf_type_fusion_config_proto",
    deps = [
        ":ccrf_type_fusion_config_proto_lib",
    ],
)

proto_library(
    name = "ccrf_type_fusion_config_proto_lib",
    srcs = ["ccrf_type_fusion_config.proto"],
)

cc_proto_library(
    name = "fused_classifier_config_proto",
    deps = [
        ":fused_classifier_config_proto_lib",
    ],
)

proto_library(
    name = "fused_classifier_config_proto_lib",
    srcs = ["fused_classifier_config.proto"],
)

cc_proto_library(
    name = "map_manager_config_proto",
    deps = [
        ":map_manager_config_proto_lib",
    ],
)

proto_library(
    name = "map_manager_config_proto_lib",
    srcs = ["map_manager_config.proto"],
)

cc_proto_library(
    name = "pbf_tracker_config_proto",
    deps = [
        ":pbf_tracker_config_proto_lib",
    ],
)

proto_library(
    name = "pbf_tracker_config_proto_lib",
    srcs = ["pbf_tracker_config.proto"],
)

cc_proto_library(
    name = "probabilistic_fusion_config_proto",
    deps = [
        ":probabilistic_fusion_config_proto_lib",
    ],
)

proto_library(
    name = "probabilistic_fusion_config_proto_lib",
    srcs = ["probabilistic_fusion_config.proto"],
)

cc_proto_library(
    name = "roi_boundary_filter_config_proto",
    deps = [
        ":roi_boundary_filter_config_proto_lib",
    ],
)

proto_library(
    name = "roi_boundary_filter_config_proto_lib",
    srcs = ["roi_boundary_filter_config.proto"],
)

cc_proto_library(
    name = "rt_proto",
    deps = [
        ":rt_proto_lib",
    ],
)

proto_library(
    name = "rt_proto_lib",
    srcs = ["rt.proto"],
)

cc_proto_library(
    name = "sensor_meta_schema_proto",
    deps = [
        ":sensor_meta_schema_proto_lib",
    ],
)

proto_library(
    name = "sensor_meta_schema_proto_lib",
    srcs = ["sensor_meta_schema.proto"],
)

cc_proto_library(
    name = "dst_type_fusion_config_proto",
    deps = [
        ":dst_type_fusion_config_proto_lib",
    ],
)

proto_library(
    name = "dst_type_fusion_config_proto_lib",
    srcs = ["dst_type_fusion_config.proto"],
)

cc_proto_library(
    name = "tracker_config_proto",
    deps = [
        ":tracker_config_proto_lib",
    ],
)

proto_library(
    name = "tracker_config_proto_lib",
    srcs = ["tracker_config.proto"],
)

cpplint()
