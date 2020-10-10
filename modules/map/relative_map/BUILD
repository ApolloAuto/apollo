load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MAP_COPTS = ["-DMODULE_NAME=\\\"map\\\""]

filegroup(
    name = "relative_map_conf",
    srcs = glob(["conf/**"]),
)

cc_library(
    name = "relative_map_lib",
    srcs = ["relative_map.cc"],
    hdrs = ["relative_map.h"],
    copts = MAP_COPTS,
    data = [
        ":relative_map_conf",
    ],
    deps = [
        ":navigation_lane_lib",
        "//modules/common/math:geometry",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/map/relative_map/common:relative_map_gflags",
        "//modules/map/relative_map/proto:navigation_cc_proto",
        "//modules/map/relative_map/proto:relative_map_config_cc_proto",
        "//modules/perception/proto:perception_obstacle_cc_proto",
    ],
)

cc_library(
    name = "navigation_lane_lib",
    srcs = ["navigation_lane.cc"],
    hdrs = ["navigation_lane.h"],
    copts = MAP_COPTS,
    deps = [
        "//cyber",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/map/relative_map/common:relative_map_gflags",
        "//modules/map/relative_map/proto:navigation_cc_proto",
        "//modules/map/relative_map/proto:relative_map_config_cc_proto",
        "//modules/perception/proto:perception_obstacle_cc_proto",
    ],
)

cc_library(
    name = "relative_map_component_lib",
    srcs = ["relative_map_component.cc"],
    hdrs = ["relative_map_component.h"],
    copts = MAP_COPTS,
    deps = [
        ":relative_map_lib",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_binary(
    name = "librelative_map_component.so",
    copts = MAP_COPTS,
    linkshared = True,
    linkstatic = False,
    deps = [":relative_map_component_lib"],
)

filegroup(
    name = "testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cc_test(
    name = "navigation_lane_test",
    size = "small",
    srcs = ["navigation_lane_test.cc"],
    data = [
        ":relative_map_conf",
        ":testdata",
    ],
    deps = [
        ":relative_map_component_lib",
        "@com_github_nlohmann_json//:json",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
