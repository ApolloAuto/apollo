load("//tools:cpplint.bzl", "cpplint")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library")
load("@com_github_grpc_grpc//bazel:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "tracking_feature_cc_proto",
    deps = [
        ":tracking_feature_proto",
    ],
)

proto_library(
    name = "tracking_feature_proto",
    srcs = ["tracking_feature.proto"],
)

cc_library(
    name = "external_feature_extractor",
    srcs = ["external_feature_extractor.cc"],
    hdrs = ["external_feature_extractor.h"],
    deps = [
        ":tracking_feature_cc_proto",
        "//cyber/common:file",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

cc_library(
    name = "project_feature",
    srcs = ["project_feature.cc"],
    hdrs = ["project_feature.h"],
    deps = [
        ":tracking_feature_cc_proto",
        "//cyber/common:file",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference/utils:inference_gemm_lib",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

cc_library(
    name = "tracking_feat_extractor",
    srcs = ["tracking_feat_extractor.cc"],
    hdrs = ["tracking_feat_extractor.h"],
    deps = [
        ":tracking_feature_cc_proto",
        "//cyber/common:file",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference/operators:perception_inference_operators",
        "//modules/perception/inference/utils:inference_gemm_lib",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

py_proto_library(
    name = "tracking_feature_py_pb2",
    deps = [
        ":tracking_feature_proto",
    ],
)

cpplint()
