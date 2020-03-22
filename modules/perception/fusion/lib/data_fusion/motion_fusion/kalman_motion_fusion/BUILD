load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kalman_motion_fusion",
    srcs = ["kalman_motion_fusion.cc"],
    hdrs = ["kalman_motion_fusion.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/geometry:basic",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/fusion/base",
        "//modules/perception/fusion/common:base_filter",
        "//modules/perception/fusion/common:kalman_filter",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/lib/registerer",
    ],
)

cc_test(
    name = "kalman_motion_fusion_test",
    size = "small",
    srcs = ["kalman_motion_fusion_test.cc"],
    deps = [
        ":kalman_motion_fusion",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
