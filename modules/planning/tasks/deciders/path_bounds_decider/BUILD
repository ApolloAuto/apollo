load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_bounds_decider",
    srcs = ["path_bounds_decider.cc"],
    hdrs = ["path_bounds_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks/deciders/utils:path_decider_obstacle_utils",
    ],
)

cc_test(
    name = "path_bounds_decider_test",
    size = "small",
    srcs = ["path_bounds_decider_test.cc"],
    deps = [
        "path_bounds_decider",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
