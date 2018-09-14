load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cnnseg",
    srcs = ["cnn_segmentation.cc"],
    hdrs = ["cnn_segmentation.h"],
    deps = [
        "//framework:cybertron",
        "//modules/common/util",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/lidar/interface",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_cluster2d",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_feature_generator",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_util",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg/proto:cnnseg_proto",
        "//modules/perception/proto:cnn_segmentation_config_lib_proto",
        "@caffe//:lib",
    ],
)

cc_library(
    name = "cnnseg_util",
    hdrs = ["util.h"],
)

cc_library(
    name = "cnnseg_feature_generator",
    srcs = ["feature_generator.cc"],
    hdrs = ["feature_generator.h"],
    deps = [
        "//framework:cybertron",
        "//modules/perception/common:pcl_util",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_util",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg/proto:cnnseg_proto",
        "@caffe//:lib",
        "@eigen",
    ],
)

cc_library(
    name = "cnnseg_cluster2d",
    hdrs = ["cluster2d.h"],
    deps = [
        "//framework:cybertron",
        "//modules/common/util:disjoint_set",
        "//modules/perception/common:pcl_util",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/lidar/segmentation/cnnseg:cnnseg_util",
        "@caffe//:lib",
    ],
)

cc_test(
    name = "cnn_segmentation_test",
    size = "small",
    srcs = [
        "cnn_segmentation_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
    ],
    deps = [
        "cnnseg",
        "//framework:cybertron",
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/obstacle/base",
        "@caffe//:lib",
        "@gtest//:main",
        "@opencv2//:core",
        "@pcl",
    ],
)

cpplint()
