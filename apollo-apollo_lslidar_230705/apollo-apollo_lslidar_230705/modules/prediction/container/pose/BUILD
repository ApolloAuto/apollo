load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pose_container",
    srcs = ["pose_container.cc"],
    hdrs = ["pose_container.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/common/math",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container",
    ],
)

cc_test(
    name = "pose_container_test",
    size = "small",
    srcs = ["pose_container_test.cc"],
    deps = [
        "//modules/prediction/container/pose:pose_container",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cpplint()
