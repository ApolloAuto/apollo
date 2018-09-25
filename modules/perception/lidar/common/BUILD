load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cloud_mask",
    srcs = [
        "cloud_mask.cc",
    ],
    hdrs = [
        "cloud_mask.h",
    ],
    deps = [
        "//modules/perception/base:point_cloud",
    ],
)

cc_test(
    name = "cloud_mask_test",
    size = "small",
    srcs = [
        "cloud_mask_test.cc",
    ],
    deps = [
        ":cloud_mask",
        "@gtest//:main",
    ],
)

cc_library(
    name = "lidar_log",
    hdrs = [
        "lidar_log.h",
    ],
    deps = [
        "//framework:cybertron",
    ],
)

cc_library(
    name = "feature_descriptor",
    hdrs = [
        "feature_descriptor.h",
    ],
    deps = [
        "//framework:cybertron",
        "//modules/perception/base:point_cloud",
    ],
)

cc_library(
    name = "lidar_frame",
    srcs = [
        "lidar_frame_pool.cc",
    ],
    hdrs = [
        "lidar_frame.h",
        "lidar_frame_pool.h",
    ],
    deps = [
        "//modules/perception/base",
        ":lidar_log",
    ],
)

cc_library(
    name = "lidar_object_util",
    srcs = [
        "lidar_object_util.cc",
    ],
    hdrs = [
        "lidar_object_util.h",
    ],
    deps = [
        "//modules/perception/base",
        ":lidar_log",
    ],
)

cpplint()
