load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "app",
    deps = [
        ":lidar_obstacle_detection",
        ":lidar_obstacle_tracking",
    ],
)

cc_library(
    name = "lidar_obstacle_detection",
    srcs = ["lidar_obstacle_detection.cc"],
    hdrs = ["lidar_obstacle_detection.h"],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/common:lidar_error_code",
        "//modules/perception/lidar/app/proto:lidar_obstacle_detection_config_cc_proto",
        "//modules/perception/lidar/lib/map_manager",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/object_filter_bank",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/lidar/lib/interface:base_pointcloud_preprocessor",
        "//modules/perception/lidar/lib/interface:base_lidar_obstacle_detection",
        "//modules/perception/lidar/lib/pointcloud_preprocessor:pointcloud_preprocessor",
        "//modules/perception/lidar/lib/detector/point_pillars_detection:point_pillars_detection",
        "//modules/perception/lidar/lib/detector/center_point_detection:center_point_detection",
        "//modules/perception/lidar/lib/detector/cnn_segmentation:cnn_segmentation",
        "//modules/perception/lidar/lib/detector/mask_pillars_detection:mask_pillars_detection",
        "//modules/perception/lidar/lib/detector/ncut_segmentation:ncut_segmentation",
        "//modules/perception/lidar/lib/scene_manager",
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "lidar_obstacle_tracking",
    srcs = ["lidar_obstacle_tracking.cc"],
    hdrs = ["lidar_obstacle_tracking.h"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "//modules/perception/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/app/proto:lidar_obstacle_tracking_config_cc_proto",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface:base_lidar_obstacle_tracking",
        "//modules/perception/lidar/lib/interface:base_classifier",
        "//modules/perception/lidar/lib/interface:base_multi_target_tracker",
        "//modules/perception/lidar/lib/classifier/fused_classifier:fused_classifier",
        "//modules/perception/lidar/lib/tracker/multi_lidar_fusion:mlf_engine",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
