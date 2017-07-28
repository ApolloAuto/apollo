load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_lidar_tracker_hm_tracker",
    srcs = [
        "hm_tracker.cc",
        "kalman_filter.cc",
        "hungarian_matcher.cc",
        "object_track.cc",
        "tracked_object.cc",
        "track_object_distance.cc",
        "feature_descriptor.cc",
    ],
    hdrs = [
        "hm_tracker.h",
        "base_filter.h",
        "base_matcher.h",
        "kalman_filter.h",
        "hungarian_matcher.h",
        "object_track.h",
        "tracked_object.h",
        "track_object_distance.h",
        "feature_descriptor.h"],
    deps = [
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/obstacle/lidar/interface:perception_obstacle_lidar_interface",
        "//modules/common:common",
        "//modules/common:log",
        "@eigen//:eigen",
        "@pcl//:pcl"],
)

cc_test(
    name = "hm_tracker_test",
    size = "small",
    srcs = [
        "hm_tracker_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception/conf:perception_config",
    ],
    deps = [
        ":perception_obstacle_lidar_tracker_hm_tracker",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/common:log",
        "@gtest//:main",
        "@pcl//:pcl"
    ]
)

cpplint()
