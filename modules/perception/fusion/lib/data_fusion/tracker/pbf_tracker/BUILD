load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pbf_tracker",
    srcs = ["pbf_tracker.cc"],
    hdrs = ["pbf_tracker.h"],
    deps = [
        "//modules/common/time",
        "//modules/perception/fusion/lib/data_fusion/existance_fusion/dst_existance_fusion",
        "//modules/perception/fusion/lib/data_fusion/motion_fusion/kalman_motion_fusion",
        "//modules/perception/fusion/lib/data_fusion/shape_fusion/pbf_shape_fusion",
        "//modules/perception/fusion/lib/data_fusion/type_fusion/dst_type_fusion",
        "//modules/perception/fusion/lib/interface",
        "//modules/perception/proto:pbf_tracker_config_proto",
    ],
)

cc_test(
    name = "pbf_tracker_test",
    size = "small",
    srcs = ["pbf_tracker_test.cc"],
    deps = [
        ":pbf_tracker",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
