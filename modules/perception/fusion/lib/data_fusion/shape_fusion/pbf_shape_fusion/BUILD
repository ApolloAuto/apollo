load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pbf_shape_fusion",
    srcs = ["pbf_shape_fusion.cc"],
    hdrs = ["pbf_shape_fusion.h"],
    deps = [
        "//modules/perception/fusion/base:scene",
        "//modules/perception/fusion/base:sensor_data_manager",
        "//modules/perception/fusion/base:track",
        "//modules/perception/fusion/lib/interface",
    ],
)

cc_test(
    name = "pbf_shape_fusion_test",
    size = "small",
    srcs = ["pbf_shape_fusion_test.cc"],
    deps = [
        ":pbf_shape_fusion",
        "//modules/perception/base:base_type",
        "//modules/perception/base:frame",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
