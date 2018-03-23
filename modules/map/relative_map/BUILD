load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "relative_map_conf",
    srcs = glob(["conf/**"]),
)

cc_library(
    name = "relative_map_lib",
    srcs = [
        "relative_map.cc",
    ],
    hdrs = [
        "relative_map.h",
        "relative_map_interface.h",
    ],
    data = [
        ":relative_map_conf",
    ],
    deps = [
        ":navigation_lane_lib",
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/math:geometry",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/map/relative_map/common:relative_map_gflags",
        "//modules/map/relative_map/proto:navigation_proto",
        "//modules/map/relative_map/proto:relative_map_config_proto",
        "//modules/perception/proto:perception_proto",
    ],
)

cc_library(
    name = "navigation_lane_lib",
    srcs = [
        "navigation_lane.cc",
    ],
    hdrs = [
        "navigation_lane.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/localization/proto:localization_proto",
        "//modules/map/relative_map/common:relative_map_gflags",
        "//modules/map/relative_map/proto:navigation_proto",
        "//modules/map/relative_map/proto:relative_map_config_proto",
        "//modules/perception/proto:perception_proto",
    ],
)

cc_binary(
    name = "relative_map",
    srcs = ["main.cc"],
    deps = [
        ":relative_map_lib",
        "//external:gflags",
    ],
)

cpplint()
