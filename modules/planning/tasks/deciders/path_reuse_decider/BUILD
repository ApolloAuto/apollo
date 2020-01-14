load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_reuse_decider",
    srcs = ["path_reuse_decider.cc"],
    hdrs = ["path_reuse_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:history",
        "//modules/planning/common:obstacle_blocking_analyzer",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/tasks/deciders:decider_base",
    ],
)

cc_test(
    name = "path_reuse_decider_test",
    size = "small",
    srcs = ["path_reuse_decider_test.cc"],
    deps = [
        "path_reuse_decider",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
