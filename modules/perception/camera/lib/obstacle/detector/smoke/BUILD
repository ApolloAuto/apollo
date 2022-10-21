load("@rules_cc//cc:defs.bzl", "cc_library")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

gpu_library(
    name = "smoke_region_output_cuda",
    srcs = ["region_output.cu"],
    hdrs = ["region_output.h"],
    deps = [
        ":smoke_object_maintainer",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/obstacle/detector/smoke/proto:smoke_cc_proto",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_library(
    name = "smoke_region_output",
    srcs = [
        "region_output.cc",
        ":smoke_region_output_cuda",
    ],
    hdrs = ["region_output.h"],
    deps = [
        ":smoke_object_maintainer",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/obstacle/detector/smoke/proto:smoke_cc_proto",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_library(
    name = "smoke_obstacle_detector",
    srcs = ["smoke_obstacle_detector.cc"],
    hdrs = ["smoke_obstacle_detector.h"],
    deps = [
        ":smoke_region_output",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/feature_extractor/tfe:external_feature_extractor",
        "//modules/perception/camera/lib/feature_extractor/tfe:project_feature",
        "//modules/perception/camera/lib/feature_extractor/tfe:tracking_feat_extractor",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/obstacle/detector/yolo:region_output",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "//modules/perception/pipeline:stage",
    ],
)

cc_library(
    name = "smoke_object_maintainer",
    srcs = ["object_maintainer.cc"],
    hdrs = ["object_maintainer.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
    ],
)

cpplint()
