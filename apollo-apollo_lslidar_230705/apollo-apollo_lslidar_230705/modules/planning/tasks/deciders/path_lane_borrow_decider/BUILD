load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_lane_borrow_decider",
    srcs = ["path_lane_borrow_decider.cc"],
    hdrs = ["path_lane_borrow_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:obstacle_blocking_analyzer",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks/deciders:decider_base",
    ],
)

cpplint()
