load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_plugin", "apollo_cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_plugin(
    name = "libgpu_down_sample.so",
    srcs = ["gpu_down_sample.cc"],
    hdrs = ["gpu_down_sample.h"],
    description = ":plugins.xml",
    copts = PERCEPTION_COPTS,
    deps = [
        ":voxel_downsample_cuda",
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/interface:base_down_sample",
        # "//modules/perception/lidar_detection:apollo_perception_lidar_detection",
        "//modules/perception/perception_plugin/lidar_detection/gpu_down_sample/proto:gpu_down_sample_proto",
    ],
)

gpu_library(
    name = "voxel_downsample_cuda",
    srcs = ["voxel_downsample.cu"],
    hdrs = ["voxel_downsample.h"],
    deps = [
        "@local_config_pcl//:pcl",
        "//modules/perception/common/base:apollo_perception_common_base",
        "@eigen",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

apollo_package(enable_source=False)
cpplint()
