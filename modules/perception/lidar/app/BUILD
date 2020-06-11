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
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:header_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/lib/utils",
        "//modules/perception/lidar/app/proto:app_proto",
        "//modules/perception/lidar/lib/detection/lidar_point_pillars:point_pillars_detection",
        "//modules/perception/lidar/lib/map_manager",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/object_filter_bank",
        "//modules/perception/lidar/lib/pointcloud_preprocessor",
        "//modules/perception/lidar/lib/scene_manager",
        "//modules/perception/proto:perception_config_schema_proto",
        "@eigen",
    ],
)

cc_library(
    name = "lidar_obstacle_segmentation",
    srcs = ["lidar_obstacle_segmentation.cc"],
    hdrs = ["lidar_obstacle_segmentation.h"],
    deps = [
        "//cyber",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:header_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/lib/utils",
        "//modules/perception/lidar/app/proto:app_proto",
        "//modules/perception/lidar/lib/map_manager",
        "//modules/perception/lidar/lib/object_builder",
        "//modules/perception/lidar/lib/object_filter_bank",
        "//modules/perception/lidar/lib/pointcloud_preprocessor",
        "//modules/perception/lidar/lib/scene_manager",
        "//modules/perception/proto:perception_config_schema_proto",
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
        "//modules/perception/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/utils",
        "//modules/perception/lidar/app/proto:app_proto",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface",
        "@eigen",
    ],
)

cc_test(
    name = "lidar_app_lidar_pipeline_test",
    size = "small",
    srcs = ["lidar_app_lidar_pipeline_test.cc"],
    deps = [
        ":app",
        "//modules/perception/common/io:io_util",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
