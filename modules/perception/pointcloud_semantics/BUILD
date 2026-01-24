load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "pointcloud_semantic_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ])
)

apollo_cc_library(
    name = "base_pointcloud_parser",
    hdrs = ["interface/base_pointcloud_parser.h",],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
    ],
)

cuda_library(
    name = "pointwise_unet_cuda",
    srcs = ["parser/pointwise_unet/pointwise_unet.cu",],
    hdrs = ["parser/pointwise_unet/pointwise_unet.h",],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        ":base_pointcloud_parser",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/pointcloud_semantics/parser/pointwise_unet/proto:pointwise_unet_proto",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "@local_config_cuda//cuda:cudart",
    ],
)

apollo_cc_library(
    name = "apollo_perception_pointcloud_semantics",
    srcs = [
        "parser/pointwise_unet/pointwise_unet.cc",
    ],
    hdrs = [
        "parser/pointwise_unet/pointwise_unet.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        ":pointwise_unet_cuda",
        ":base_pointcloud_parser",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/pointcloud_semantics/parser/pointwise_unet/proto:pointwise_unet_proto",
        "@local_config_pcl//:pcl",
    ],
)

apollo_component(
    name = "libpointcloud_semantic_component.so",
    srcs = ["pointcloud_semantic_component.cc"],
    hdrs = ["pointcloud_semantic_component.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    linkopts = ["-lleveldb"],
    deps = [
        "//cyber",
        ":apollo_perception_pointcloud_semantics",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/pointcloud_semantics/proto:pointcloud_semantic_component_config_cc_proto",
        "@boost",
    ],
)

apollo_package(enable_source=False)
cpplint()
