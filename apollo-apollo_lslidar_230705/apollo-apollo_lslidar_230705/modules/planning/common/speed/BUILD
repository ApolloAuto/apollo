load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "st_point",
    srcs = ["st_point.cc"],
    hdrs = ["st_point.h"],
    deps = [
        "//modules/common/math",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "st_boundary",
    srcs = ["st_boundary.cc"],
    hdrs = ["st_boundary.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":st_point",
        "//cyber",
        "//modules/planning/common:planning_gflags",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "st_boundary_test",
    size = "small",
    srcs = ["st_boundary_test.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":st_boundary",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "speed_data",
    srcs = ["speed_data.cc"],
    hdrs = ["speed_data.h"],
    deps = [
        ":st_point",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
