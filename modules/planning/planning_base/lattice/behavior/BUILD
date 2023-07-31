load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "feasible_region",
    srcs = ["feasible_region.cc"],
    hdrs = ["feasible_region.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/planning/planning_base/common:planning_gflags",
    ],
)

cc_library(
    name = "path_time_graph",
    srcs = ["path_time_graph.cc"],
    hdrs = ["path_time_graph.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:obstacle",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/proto:lattice_structure_cc_proto",
        "//modules/planning/planning_base/reference_line",
    ],
)

cc_library(
    name = "prediction_querier",
    srcs = ["prediction_querier.cc"],
    hdrs = ["prediction_querier.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/math",
        "//modules/planning/planning_base/common:obstacle",
        "//modules/planning/planning_base/proto:lattice_structure_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
    ],
)

apollo_package()
cpplint()
