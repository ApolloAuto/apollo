load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ground_service_detector",
    srcs = ["ground_service_detector.cc"],
    hdrs = ["ground_service_detector.h"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/perception/common:util",
        "//modules/perception/common/base:base_type",
        "//modules/perception/common/lib/thread",
        "//modules/perception/common/lidar/common:lidar_frame",
        "//modules/perception/common/lidar/common:lidar_point_label",
        "//modules/perception/common/lidar/scene_manager",
        "//modules/perception/common/lidar/scene_manager:scene_service",
        "//modules/perception/common/lidar/scene_manager/ground_service",
        "//modules/perception/common/lidar/scene_manager/ground_service:ground_struct",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "//modules/perception/pointcloud_ground_detection/ground_detector/proto:ground_service_detector_config_cc_proto",
        "//modules/perception/pointcloud_ground_detection/interface:base_ground_detector",
        "@eigen",
    ],
)

# cc_test(
#     name = "ground_service_detector_test",
#     size = "small",
#     srcs = ["ground_service_detector_test.cc"],
#     deps = [
#         ":ground_service_detector",
#         "//modules/perception/common:perception_gflags",
#         "//modules/perception/common/algorithm/io:io_util",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

apollo_package()
cpplint()
