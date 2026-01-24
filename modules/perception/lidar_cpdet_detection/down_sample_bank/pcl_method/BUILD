load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_plugin", "apollo_cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_plugin(
    name = "libpcl_pc_downsample.so",
    srcs = ["pcl_method.cc"],
    hdrs = ["pcl_method.h"],
    description = ":plugins.xml",
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/interface:base_down_sample",
        "//modules/perception/lidar_cpdet_detection:apollo_perception_lidar_cpdetection",
        "//modules/perception/lidar_cpdet_detection/down_sample_bank/pcl_method/proto:pcl_method_proto",
    ],
)

apollo_package()
cpplint()
