load("@rules_cc//cc:defs.bzl", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
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
        "@eigen",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "@opencv//:highgui",
        "@local_config_cuda//cuda:cudart",
    ]+ select({
        "@platforms//cpu:x86_64": ["@paddleinference-x86_64//:paddleinference_lib"],
        "@platforms//cpu:aarch64": ["@paddleinference-aarch64//:paddleinference_lib"],
    })+ select({
        "@platforms//cpu:x86_64": ["@caddn_infer_op-x86_64//:caddn_infer_op"],
        "@platforms//cpu:aarch64": ["@caddn_infer_op-aarch64//:caddn_infer_op"],
    }),
)

cpplint()
