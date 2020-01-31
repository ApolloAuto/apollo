load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    deps = [
        ":cloud_mask",
        ":lidar_error_code",
        ":lidar_frame",
        ":lidar_log",
        ":lidar_object_util",
        ":lidar_point_label",
        ":lidar_timer",
        ":pcl_util",
    ],
)

cc_library(
    name = "cloud_mask",
    srcs = ["cloud_mask.cc"],
    hdrs = ["cloud_mask.h"],
    deps = [
        "//modules/perception/base:point_cloud",
    ],
)

cc_test(
    name = "cloud_mask_test",
    size = "small",
    srcs = ["cloud_mask_test.cc"],
    deps = [
        ":cloud_mask",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "lidar_log",
    hdrs = ["lidar_log.h"],
    deps = [
        "//cyber",
    ],
)

cc_library(
    name = "feature_descriptor",
    hdrs = ["feature_descriptor.h"],
    deps = [
        "//cyber",
        "//modules/perception/base:point_cloud",
    ],
)

cc_library(
    name = "lidar_frame",
    srcs = ["lidar_frame_pool.cc"],
    hdrs = [
        "lidar_frame.h",
        "lidar_frame_pool.h",
    ],
    deps = [
        ":lidar_log",
        "//modules/perception/base",
    ],
)

cc_library(
    name = "lidar_object_util",
    srcs = ["lidar_object_util.cc"],
    hdrs = ["lidar_object_util.h"],
    deps = [
        ":lidar_log",
        "//modules/perception/base",
    ],
)

cc_library(
    name = "lidar_point_label",
    hdrs = ["lidar_point_label.h"],
)

cc_library(
    name = "lidar_timer",
    hdrs = ["lidar_timer.h"],
)

cc_library(
    name = "lidar_error_code",
    hdrs = ["lidar_error_code.h"],
)

cc_library(
    name = "pcl_util",
    hdrs = ["pcl_util.h"],
    deps = [
        ":lidar_log",
        "//modules/perception/base",
        "@pcl",
    ],
)

cc_library(
    name = "object_sequence",
    srcs = ["object_sequence.cc"],
    hdrs = ["object_sequence.h"],
    deps = [
        "//cyber",
        "//modules/perception/base:object",
        "//modules/perception/lidar/common",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
