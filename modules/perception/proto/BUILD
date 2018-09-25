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
    name = "perception_config_schema_proto",
    deps = [
        ":perception_config_schema_proto_lib",
    ],
)

proto_library(
    name = "perception_config_schema_proto_lib",
    srcs = [
        "perception_config_schema.proto",
    ],
    deps = [
    ],
)

cc_proto_library(
    name = "perception_ultrasonic_proto",
    deps = [
        ":perception_ultrasonic_proto_lib",
    ],
)

proto_library(
    name = "perception_ultrasonic_proto_lib",
    srcs = [
        "perception_ultrasonic.proto",
    ],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "sensor_meta_schema_proto",
    deps = [
        ":sensor_meta_schema_proto_lib",
    ],
)

proto_library(
    name = "sensor_meta_schema_proto_lib",
    srcs = [
        "sensor_meta_schema.proto",
    ],
    deps = [
    ],
)

cpplint()
