load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_cc_library(
    name = "graph_segmentation",
    srcs = ["graph_segmentation.cc", "background_map.cc"],
    hdrs = ["graph_segmentation.h", "background_map.h"],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/lidar_detection:apollo_perception_lidar_detection",
        "//modules/perception/lidar_segmentation/common:lidar_segmention_common",
        "//modules/perception/lidar_segmentation/segmentor/graph_segmentation/proto:graph_segmentation_config_cc_proto",
        "@local_config_pcl//:pcl",
        "@eigen",
    ],
)

apollo_package(enable_source=False)
cpplint()
