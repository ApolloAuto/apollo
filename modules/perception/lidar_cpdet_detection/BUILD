load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "lidar_cpdet_detection_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ]),
)

apollo_cc_library(
    name = "base_cpdetector",
    hdrs = ["interface/base_cpdetector.h",],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
    ],
)

gpu_library(
    name = "cpdet_cuda",
    srcs = ["detector/cpdet/cpdet.cu"],
    hdrs = ["detector/cpdet/cpdet.h"],
    deps = [
        ":base_cpdetector",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/interface:base_down_sample",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/lidar_cpdet_detection/detector/cpdet/proto:cpdet_model_param_cc_proto",
        "@local_config_cuda//cuda:cudart",
    ]
)

apollo_cc_library(
    name = "apollo_perception_lidar_cpdetection",
    srcs = [
        "detector/cpdet/cpdet.cc",
    ],
    hdrs = [
        "detector/cpdet/cpdet.h",
        "interface/base_cpdetector.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        ":cpdet_cuda",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/interface:base_down_sample",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "//modules/perception/lidar_cpdet_detection/down_sample_bank/pcl_method/proto:pcl_method_proto",
        "//modules/perception/lidar_cpdet_detection/proto:lidar_cpdet_detection_component_config_cc_proto",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest",
        "@eigen",
        "@local_config_pcl//:pcl",
        "@libtorch_gpu",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:migraphx",
    ]),
)

apollo_component(
    name = "liblidar_cpdet_detection_component.so",
    srcs = ["lidar_cpdetection_component.cc"],
    hdrs = ["lidar_cpdetection_component.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    linkopts = ["-lleveldb"],
    deps = [
        ":apollo_perception_lidar_cpdetection",
        "//cyber",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/lidar_cpdet_detection/proto:lidar_cpdet_detection_component_config_cc_proto",
    ],
)

apollo_package(enable_source=False)

cpplint()
