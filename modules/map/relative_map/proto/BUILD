load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "navigation_proto",
    deps = [
        ":navigation_proto_lib",
    ],
)

proto_library(
    name = "navigation_proto_lib",
    srcs = ["navigation.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/localization/proto:localization_proto_lib",
        "//modules/map/proto:map_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
    ],
)

cc_proto_library(
    name = "relative_map_config_proto",
    deps = [
        ":relative_map_config_proto_lib",
    ],
)

proto_library(
    name = "relative_map_config_proto_lib",
    srcs = ["relative_map_config.proto"],
)

cc_proto_library(
    name = "navigator_config_proto",
    deps = [
        ":navigator_config_proto_lib",
    ],
)

proto_library(
    name = "navigator_config_proto_lib",
    srcs = ["navigator_config.proto"],
)

py_proto(
    name = "navigator_config_pb2",
    src = "navigator_config.proto",
)

py_proto(
    name = "relative_map_config_pb2",
    src = "relative_map_config.proto",
)

py_proto(
    name = "navigation_pb2",
    src = "navigation.proto",
    deps = [
        "//modules/common/proto:header_pb2",
        "//modules/common/proto:pnc_point_pb2",
        "//modules/localization/proto:localization_pb2",
        "//modules/map/proto:map_pb2",
        "//modules/perception/proto:perception_obstacle_pb2",
    ],
)
