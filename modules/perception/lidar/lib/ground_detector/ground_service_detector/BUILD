load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ground_service_detector",
    srcs = ["ground_service_detector.cc"],
    hdrs = ["ground_service_detector.h"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/perception/base:base_type",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/thread",
        "//modules/perception/lidar/common:lidar_frame",
        "//modules/perception/lidar/common:lidar_point_label",
        "//modules/perception/lidar/lib/ground_detector/ground_service_detector/proto:ground_service_detector_config_proto",
        "//modules/perception/lidar/lib/interface:base_ground_detector",
        "//modules/perception/lidar/lib/scene_manager",
        "//modules/perception/lidar/lib/scene_manager:scene_service",
        "//modules/perception/lidar/lib/scene_manager/ground_service",
        "//modules/perception/lidar/lib/scene_manager/ground_service:ground_struct",
        "//modules/perception/proto:perception_config_schema_proto",
        "@eigen",
    ],
)

cc_test(
    name = "ground_service_detector_test",
    size = "small",
    srcs = ["ground_service_detector_test.cc"],
    deps = [
        ":ground_service_detector",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/io:io_util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
