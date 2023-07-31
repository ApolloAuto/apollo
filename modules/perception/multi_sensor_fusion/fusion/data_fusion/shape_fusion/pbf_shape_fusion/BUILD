load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pbf_shape_fusion",
    srcs = ["pbf_shape_fusion.cc"],
    hdrs = ["pbf_shape_fusion.h"],
    deps = [
        "//modules/perception/multi_sensor_fusion/base:scene",
        "//modules/perception/multi_sensor_fusion/base:sensor_data_manager",
        "//modules/perception/multi_sensor_fusion/base:track",
        "//modules/perception/multi_sensor_fusion/interface",
    ],
)

cc_test(
    name = "pbf_shape_fusion_test",
    size = "small",
    srcs = ["pbf_shape_fusion_test.cc"],
    deps = [
        ":pbf_shape_fusion",
        "//modules/perception/common/base:base_type",
        "//modules/perception/common/base:frame",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
