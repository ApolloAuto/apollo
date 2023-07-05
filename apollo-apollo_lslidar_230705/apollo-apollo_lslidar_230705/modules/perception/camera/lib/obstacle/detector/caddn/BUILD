load("@rules_cc//cc:defs.bzl", "cc_library")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "caddn_obstacle_detector",
    srcs = ["caddn_obstacle_detector.cc"],
    hdrs = ["caddn_obstacle_detector.h"],
    deps = [
        "//cyber/common:file",
        "//modules/common/util:perf_util",
        "//modules/perception/base",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/feature_extractor/tfe:external_feature_extractor",
        "//modules/perception/camera/lib/feature_extractor/tfe:project_feature",
        "//modules/perception/camera/lib/feature_extractor/tfe:tracking_feat_extractor",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "@paddleinference//:paddleinference_lib",
        "@caddn_infer_op//:caddn_infer_op",
        "@eigen",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "@opencv//:highgui",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cpplint()
