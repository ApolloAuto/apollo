load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "path_generation",
    srcs = ["path_generation.cc"],
    hdrs = ["path_generation.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/task_base:task",
        "//modules/planning/planning_base/task_base/common/path_util:path_util"
    ],
)

cc_library(
    name = "decider_base",
    srcs = ["decider.cc"],
    hdrs = ["decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/planning/planning_base/common:frame",
        "//modules/planning/planning_base/task_base:task",
    ],
)

cc_library(
    name = "speed_optimizer",
    srcs = ["speed_optimizer.cc"],
    hdrs = ["speed_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/planning/planning_base/common:path_decision",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:reference_line_info",
        "//modules/planning/planning_base/common:speed_limit",
        "//modules/planning/planning_base/common:st_graph_data",
        "//modules/planning/planning_base/common/path:path_data",
        "//modules/planning/planning_base/common/speed:speed_data",
        "//modules/planning/planning_base/task_base:task",
        "@eigen",
    ],
)

cc_library(
    name = "trajectory_optimizer",
    srcs = ["trajectory_optimizer.cc"],
    hdrs = ["trajectory_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common/trajectory:discretized_trajectory",
        "//modules/planning/planning_base/task_base:task",
    ],
    alwayslink =True
)

apollo_package()
cpplint()
