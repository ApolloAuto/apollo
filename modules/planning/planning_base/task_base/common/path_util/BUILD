load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_util",
    srcs = ["path_assessment_decider_util.cc",
                  "path_bounds_decider_util.cc",
                  "path_optimizer_util.cc"],
    hdrs = ["path_assessment_decider_util.h",
                  "path_bounds_decider_util.h",
                  "path_optimizer_util.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/common:reference_line_info",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common/path:discretized_path",
        "//modules/planning/planning_base/common/path:frenet_frame_path",
        "//modules/planning/planning_base/common/path:path_data",
        "//modules/planning/planning_base/proto:piecewise_jerk_path_config_cc_proto",
         "//modules/planning/planning_base/math/piecewise_jerk:piecewise_jerk_path_problem",
         "//modules/planning/planning_base/common/trajectory1d:piecewise_jerk_trajectory1d"
    ],
)

apollo_package()
cpplint()
