load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pbf_tracker",
    srcs = ["pbf_tracker.cc"],
    hdrs = ["pbf_tracker.h"],
    deps = [
        "//modules/common/util:string_util",
        "//modules/perception/fusion/lib/data_fusion/existence_fusion/dst_existence_fusion",
        "//modules/perception/fusion/lib/data_fusion/motion_fusion/kalman_motion_fusion",
        "//modules/perception/fusion/lib/data_fusion/shape_fusion/pbf_shape_fusion",
        "//modules/perception/fusion/lib/data_fusion/type_fusion/dst_type_fusion",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/proto:pbf_tracker_config_cc_proto",
    ],
)

cc_test(
    name = "pbf_tracker_test",
    size = "small",
    srcs = ["pbf_tracker_test.cc"],
    copts = ["-fno-access-control"],
    deps = [
        ":pbf_tracker",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
