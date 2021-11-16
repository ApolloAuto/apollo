load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_assessment_decider",
    srcs = ["path_assessment_decider.cc"],
    hdrs = ["path_assessment_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks/deciders/path_bounds_decider",
        "//modules/planning/tasks/deciders/utils:path_decider_obstacle_utils",
    ],
)

cpplint()
