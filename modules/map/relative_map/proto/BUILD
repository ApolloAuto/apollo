load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "navigation_proto",
    deps = [
        ":navigation_proto_lib",
    ],
)

proto_library(
    name = "navigation_proto_lib",
    srcs = [
        "navigation.proto",
    ],
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
    srcs = [
        "relative_map_config.proto",
    ],
)

cpplint()
