load("//tools:cpplint.bzl", "cpplint")

load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_lidar_segmention_cnnseg",
    srcs = ["cnn_segmentation.cc"],
    hdrs = ["cnn_segmentation.h"],
    deps = [
        "@caffe//:caffe",
        "//modules/common:log",
        "//modules/perception/common:perception_common",
        "//modules/common/util:util",
        "//modules/perception/lib/pcl_util:pcl_util",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/lidar/interface:perception_obstacle_lidar_interface",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg/proto:cnnseg_proto",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_util",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_feature_generator",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_cluster2d",
        "//modules/perception/lib/base:base",
        "//modules/perception/lib/config_manager:config_manager",
    ],
)

cc_library(
    name = "cnnseg_util",
    hdrs = ["util.h"],
    deps = ["@opencv2//:core"],
)

cc_library(
    name = "cnnseg_feature_generator",
    srcs = ["feature_generator.cc"],
    hdrs = ["feature_generator.h"],
    deps = [
         "@caffe//:caffe",
         "//modules/common:log",
         "//modules/perception/lib/pcl_util:pcl_util",
         "@eigen//:eigen",
         "//modules/perception/obstacle/lidar/segmentation/cnnseg/proto:cnnseg_proto",
         "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_util"
    ],
)

cc_library(
    name = "cnnseg_cluster2d",
    hdrs = ["cluster2d.h"],
    deps = [
         "@caffe//:caffe",
         "//modules/common:log",
         "//modules/perception/lib/pcl_util:pcl_util",
         "//modules/perception/obstacle/base:perception_obstacle_base",
         "//modules/perception/obstacle/common:perception_obstacle_common",
         "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_util",
    ],
)

cc_test(
    name = "cnnseg_test",
    size = "small",
    srcs = [
        "cnnseg_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception/conf:perception_config",
        "//modules/perception:perception_model",
    ],
    deps = [
        "@gtest//:main",
        "@pcl//:pcl",
        "@opencv2//:core",
        "//modules/common:log",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "perception_obstacle_lidar_segmention_cnnseg",
    ],
)

cpplint()