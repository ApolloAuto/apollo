load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "external_feature_extractor",
    srcs = ["external_feature_extractor.cc"],
    hdrs = ["external_feature_extractor.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera_tracking/feature_extract/proto:model_param_cc_proto",
        "//modules/perception/camera_tracking/interface:base_feature_extractor",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/inference:inference_factory",
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference:model_util",
        "//modules/perception/common/inference/utils:inference_gemm_lib",
        "//modules/perception/common/inference/utils:inference_resize_lib",
        "//modules/perception/common/inference/utils:inference_util_lib",
    ],
    alwayslink = True,
)

cc_library(
    name = "project_feature",
    srcs = ["project_feature.cc"],
    hdrs = ["project_feature.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera_tracking/feature_extract/proto:model_param_cc_proto",
        "//modules/perception/camera_tracking/interface:base_feature_extractor",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/inference:inference_factory",
        "//modules/perception/common/inference:model_util",
        "//modules/perception/common/inference/utils:inference_gemm_lib",
        "//modules/perception/common/inference/utils:inference_util_lib",
    ],
    alwayslink = True,
)

cc_library(
    name = "tracking_feat_extractor",
    srcs = ["tracking_feat_extractor.cc"],
    hdrs = ["tracking_feat_extractor.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera_tracking/interface:base_feature_extractor",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/inference/utils:inference_gemm_lib",
        "//modules/perception/common/inference:inference_factory",
        "//modules/perception/common/inference/operators:perception_inference_operators",
        "//modules/perception/common/inference/utils:inference_util_lib",
        "//modules/perception/common/lib/registerer:registerer",
        "//modules/perception/camera_tracking/proto:tracking_feature_cc_proto",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
