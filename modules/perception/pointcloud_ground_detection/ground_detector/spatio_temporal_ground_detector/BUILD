load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "spatio_temporal_ground_detector",
    srcs = ["spatio_temporal_ground_detector.cc"],
    hdrs = ["spatio_temporal_ground_detector.h"],
    copts = select(
        {
            "@platforms//cpu:x86_64": ["-msse4.1"],
            "@platforms//cpu:aarch64": ["-march=armv8-a+fp+simd+crc"],
        },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
    deps = [
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/common/lib/registerer",
        "@eigen",
        "//modules/perception/common:util",
        "//modules/perception/common/algorithm/i_lib",
        "//modules/perception/common/algorithm/i_lib/pc:i_ground",
        "//modules/perception/common/algorithm/i_lib/pc:i_struct_s",
        "//modules/perception/common/algorithm/i_lib/pc:i_util",
        "//modules/perception/common/algorithm/point_cloud_processing",
        "//modules/perception/common/lidar/common",
        "//modules/perception/common/lidar/scene_manager",
        "//modules/perception/common/lidar/scene_manager/ground_service",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "//modules/perception/pointcloud_ground_detection/ground_detector/proto:spatio_temporal_ground_detector_config_cc_proto",
        "//modules/perception/pointcloud_ground_detection/interface:base_ground_detector",
    ],
    alwayslink = True,
)

# cc_test(
#     name = "spatio_temporal_ground_detector_test",
#     size = "small",
#     srcs = ["spatio_temporal_ground_detector_test.cc"],
#     copts = ["-msse4.1"],
#     deps = [
#         ":spatio_temporal_ground_detector",
#         "@com_google_googletest//:gtest_main",
#         "@local_config_pcl//:pcl",
#     ],
# )

apollo_package()
cpplint()
