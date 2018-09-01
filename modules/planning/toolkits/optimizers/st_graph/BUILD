load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "st_graph_data",
    srcs = [
        "st_graph_data.cc",
    ],
    hdrs = [
        "st_graph_data.h",
    ],
    deps = [
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:speed_limit",
        "//modules/planning/common/speed:st_boundary",
    ],
)

cc_library(
    name = "st_boundary_mapper",
    srcs = [
        "speed_limit_decider.cc",
        "st_boundary_mapper.cc",
    ],
    hdrs = [
        "speed_limit_decider.h",
        "st_boundary_mapper.h",
    ],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/configs/proto:vehicle_config_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
        "//modules/map/proto:map_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:speed_limit",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:st_boundary",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/proto:st_boundary_config_proto",
        "//modules/planning/reference_line",
    ],
)

cc_test(
    name = "speed_limit_decider_test",
    size = "small",
    srcs = [
        "speed_limit_decider_test.cc",
    ],
    data = [
        "//modules/map:map_data",
        "//modules/planning:planning_testdata",
    ],
    deps = [
        ":st_boundary_mapper",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cc_test(
    name = "st_boundary_mapper_test",
    size = "small",
    srcs = [
        "st_boundary_mapper_test.cc",
    ],
    data = [
        "//modules/map:map_data",
        "//modules/planning:planning_testdata",
    ],
    deps = [
        ":st_boundary_mapper",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cc_test(
    name = "st_graph_data_test",
    size = "small",
    srcs = [
        "st_graph_data_test.cc",
    ],
    deps = [
        ":st_graph_data",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cpplint()
