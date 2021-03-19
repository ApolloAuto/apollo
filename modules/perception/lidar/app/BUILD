load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "app",
    deps = [
        ":lidar_obstacle_detection",
        ":lidar_obstacle_segmentation",
        ":lidar_obstacle_tracking",
    ],
)

cc_library(
    name = "lidar_obstacle_detection",
    srcs = ["lidar_obstacle_detection.cc"],
    hdrs = ["lidar_obstacle_detection.h"],
    deps = [
        "//cyber",
        "//modules/common/util:perf_util",
        "//modules/perception/lidar/app/proto:lidar_obstacle_detection_config_cc_proto",
        "//modules/perception/lidar/lib/detection/lidar_point_pillars:point_pillars_detection",
        "//modules/perception/lidar/lib/pointcloud_preprocessor",
        "@eigen",
    ],
)

cc_library(
    name = "lidar_obstacle_segmentation",
    srcs = ["lidar_obstacle_segmentation.cc"],
    hdrs = ["lidar_obstacle_segmentation.h"],
    deps = [
        "//cyber",
        "//modules/common/util:perf_util",
        "//modules/perception/lidar/app/proto:lidar_obstacle_segmentation_config_cc_proto",
        "//modules/perception/lidar/lib/map_manager",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/object_filter_bank",
        "//modules/perception/lidar/lib/pointcloud_preprocessor",
        "//modules/perception/lidar/lib/scene_manager",
        "@eigen",
    ],
)

cc_library(
    name = "lidar_obstacle_tracking",
    srcs = ["lidar_obstacle_tracking.cc"],
    hdrs = ["lidar_obstacle_tracking.h"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/common/util:perf_util",
        "//modules/perception/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/app/proto:lidar_obstacle_tracking_config_cc_proto",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface:base_classifier",
        "//modules/perception/lidar/lib/interface:base_multi_target_tracker",
        "@eigen",
    ],
)

cpplint()
