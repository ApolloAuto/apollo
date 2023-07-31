load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    alwayslink = True,
    deps = [
        ":cloud_mask",
        ":config_util",
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
    alwayslink = True,
    hdrs = ["cloud_mask.h"],
    deps = [
        "//modules/perception/common/base:point_cloud",
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
    linkstatic = True,
)

cc_library(
    name = "config_util",
    srcs = ["config_util.cc"],
    alwayslink = True,
    hdrs = ["config_util.h"],
    deps = [],
)

cc_library(
    name = "lidar_log",
    hdrs = ["lidar_log.h"],
    alwayslink = True,
    deps = [
        "//cyber",
    ],
)

cc_library(
    name = "feature_descriptor",
    hdrs = ["feature_descriptor.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/base:point_cloud",
    ],
)

cc_library(
    name = "lidar_frame",
    srcs = ["lidar_frame_pool.cc"],
    hdrs = [
        "lidar_frame.h",
        "lidar_frame_pool.h",
    ],
    alwayslink = True,
    deps = [
        ":lidar_log",
        "//modules/perception/common/base",
    ],
)

cc_library(
    name = "lidar_object_util",
    srcs = ["lidar_object_util.cc"],
    hdrs = ["lidar_object_util.h"],
    alwayslink = True,
    deps = [
        ":lidar_log",
        "//modules/perception/common/base",
    ],
)

cc_library(
    name = "lidar_point_label",
    hdrs = ["lidar_point_label.h"],
    alwayslink = True,
)

cc_library(
    name = "lidar_timer",
    hdrs = ["lidar_timer.h"],
    alwayslink = True,
)

cc_library(
    name = "lidar_error_code",
    hdrs = ["lidar_error_code.h"],
    alwayslink = True,
)

cc_library(
    name = "pcl_util",
    hdrs = ["pcl_util.h"],
    alwayslink = True,
    deps = [
        ":lidar_log",
        "//modules/perception/common/base",
        "@local_config_pcl//:pcl",
    ],
)

cc_library(
    name = "object_sequence",
    srcs = ["object_sequence.cc"],
    hdrs = ["object_sequence.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/base:object",
        "//modules/perception/common/lidar/common",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "pointcloud__util",
    hdrs = ["pointcloud_util.h"],
    alwayslink = True,
    deps = [
        "@local_config_pcl//:pcl",
    ],
)

apollo_package()
cpplint()
