load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "navi_path_decider",
    srcs = [
        "navi_path_decider.cc",
    ],
    hdrs = [
        "navi_path_decider.h",
    ],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/map/proto:map_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/tasks:task",
        "//modules/planning/navi/common:common",
        "//modules/planning/navi/decider:navi_obstacle_decider",
        "@glog",
    ],
)

cc_library(
    name = "navi_speed_decider",
    srcs = [
        "navi_speed_decider.cc",
        "navi_speed_ts_graph.cc",
    ],
    hdrs = [
        "navi_speed_decider.h",
        "navi_speed_ts_graph.h",
    ],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/map/proto:map_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/navi/decider:navi_obstacle_decider",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/tasks:task",
        "@glog",
    ],
)

cc_library(
    name = "navi_obstacle_decider",
    srcs = [
        "navi_obstacle_decider.cc",
    ],
    hdrs = [
        "navi_obstacle_decider.h",
    ],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/map/proto:map_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/tasks:task",
        "//modules/common/math:path_matcher",
        "@glog",
    ],
)

cc_test(
    name = "navi_decider_test",
    size = "small",
    srcs = [
        "navi_path_decider_test.cc",
        "navi_speed_decider_test.cc",
        "navi_speed_ts_graph_test.cc",
        "navi_obstacle_decider_test.cc",
    ],
    data = ["//modules/planning:planning_testdata"],
    deps = [
        ":navi_path_decider",
        ":navi_speed_decider",
        ":navi_obstacle_decider",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cpplint()
