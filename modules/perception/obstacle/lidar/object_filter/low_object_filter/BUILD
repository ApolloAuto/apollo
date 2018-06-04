load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "low_object_filter",
    srcs = ["low_object_filter.cc"],
    hdrs = ["low_object_filter.h"],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/lidar/interface",
        "//modules/perception/proto:low_object_filter_config_lib_proto",
        "@eigen",
    ],
)

cc_test(
    name = "low_object_filter_test",
    size = "small",
    srcs = [
        "low_object_filter_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
    ],
    deps = [
        ":low_object_filter",
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/obstacle/common",
        "@gtest//:main",
    ],
)

cpplint()
