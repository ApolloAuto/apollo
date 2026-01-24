load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_plugin", "apollo_cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_plugin(
    name = "libpcl_down_sample.so",
    srcs = ["pcl_down_sample.cc"],
    hdrs = ["pcl_down_sample.h"],
    description = ":plugins.xml",
    copts = PERCEPTION_COPTS,
    deps = [
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/interface:base_down_sample",
        "//modules/perception/lidar_detection:apollo_perception_lidar_detection",
        "//modules/perception/lidar_detection/down_sample_bank/pcl_down_sample/proto:pcl_down_sample_proto",
    ],
)

apollo_package()
cpplint()
