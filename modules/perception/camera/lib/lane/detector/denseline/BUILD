load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//third_party/gpus:common.bzl", "if_cuda", "if_rocm")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "denseline_lane_detector",
    srcs = ["denseline_lane_detector.cc"],
    hdrs = ["denseline_lane_detector.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/inference:inference_factory",
    ] + if_cuda([
        "//modules/perception/inference/tensorrt:rt_net",
    ]) + if_rocm([
        "//modules/perception/inference/migraphx:mi_net",
    ]) + [
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:denseline_cc_proto",
    ],
)

cpplint()
