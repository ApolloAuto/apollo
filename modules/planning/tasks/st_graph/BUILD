load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "st_boundary",
    srcs = [
        "st_boundary.cc",
    ],
    hdrs = [
        "st_boundary.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/math",
        "//modules/planning/common/speed:st_point",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_library(
    name = "st_graph_data",
    srcs = [
        "st_graph_data.cc",
    ],
    hdrs = [
        "st_graph_data.h",
    ],
    deps = [
        ":st_boundary",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:speed_limit",
    ],
)

cc_library(
    name = "st_boundary_mapper",
    srcs = [
        "st_boundary_mapper.cc",
    ],
    hdrs = [
        "st_boundary_mapper.h",
    ],
    deps = [
        ":st_boundary",
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
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/math:double",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
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
    name = "st_boundary_test",
    size = "small",
    srcs = [
        "st_boundary_test.cc",
    ],
    deps = [
        ":st_boundary",
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
        ":st_boundary",
        ":st_graph_data",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cpplint()
