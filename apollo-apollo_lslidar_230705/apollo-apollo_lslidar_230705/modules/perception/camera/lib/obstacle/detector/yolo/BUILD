load("@rules_cc//cc:defs.bzl", "cc_library")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

gpu_library(
    name = "region_output_cuda",
    srcs = ["region_output.cu"],
    hdrs = ["region_output.h"],
    deps = [
        ":object_maintainer",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/proto:yolo_cc_proto",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_library(
    name = "region_output",
    srcs = [
        "region_output.cc",
    ],
    hdrs = ["region_output.h"],
    deps = [
        ":object_maintainer",
        ":region_output_cuda",
        "//cyber",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_library(
    name = "yolo_obstacle_detector",
    srcs = ["yolo_obstacle_detector.cc"],
    hdrs = ["yolo_obstacle_detector.h"],
    deps = [
        ":region_output",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/feature_extractor/tfe:external_feature_extractor",
        "//modules/perception/camera/lib/feature_extractor/tfe:project_feature",
        "//modules/perception/camera/lib/feature_extractor/tfe:tracking_feat_extractor",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:yolo_obstacle_detector_config_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "object_maintainer",
    srcs = ["object_maintainer.cc"],
    hdrs = ["object_maintainer.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
    ],
)

cpplint()
