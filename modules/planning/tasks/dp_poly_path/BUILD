load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dp_poly_path",
    srcs = [
        "dp_road_graph.cc",
        "trajectory_cost.cc",
    ],
    hdrs = [
        "comparable_cost.h",
        "dp_road_graph.h",
        "trajectory_cost.h",
    ],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/map/proto:map_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:planning_thread_pool",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/math/curve1d:polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
        "//modules/planning/proto:dp_poly_path_config_proto",
        "//modules/planning/reference_line",
        "@eigen",
    ],
)

cc_library(
    name = "dp_poly_path_optimizer",
    srcs = [
        "dp_poly_path_optimizer.cc",
    ],
    hdrs = [
        "dp_poly_path_optimizer.h",
    ],
    deps = [
        ":dp_poly_path",
        "//modules/planning/tasks:path_optimizer",
    ],
)

cc_test(
    name = "comparable_cost_test",
    size = "small",
    srcs = [
        "comparable_cost_test.cc",
    ],
    deps = [
        ":dp_poly_path",
        "@gtest//:main",
    ],
)

cc_test(
    name = "trajectory_cost_test",
    size = "small",
    srcs = [
        "trajectory_cost_test.cc",
    ],
    deps = [
        ":dp_poly_path",
        "@gtest//:main",
    ],
)

cpplint()
