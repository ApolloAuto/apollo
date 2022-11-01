load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "discretized_path",
    srcs = ["discretized_path.cc"],
    hdrs = ["discretized_path.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
    ],
)

cc_test(
    name = "discretized_path_test",
    size = "small",
    srcs = ["discretized_path_test.cc"],
    deps = [
        ":discretized_path",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "frenet_frame_path",
    srcs = ["frenet_frame_path.cc"],
    hdrs = ["frenet_frame_path.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/math",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
    ],
)

cc_test(
    name = "frenet_frame_path_test",
    size = "small",
    srcs = ["frenet_frame_path_test.cc"],
    deps = [
        ":frenet_frame_path",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_library(
    name = "path_data",
    srcs = ["path_data.cc"],
    hdrs = ["path_data.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":discretized_path",
        ":frenet_frame_path",
        "//modules/common/math",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/reference_line",
    ],
)

cpplint()
