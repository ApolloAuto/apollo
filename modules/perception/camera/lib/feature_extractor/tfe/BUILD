load("//tools:cpplint.bzl", "cpplint")
load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "tracking_feature_proto",
    deps = [
        ":tracking_feature_proto_lib",
    ],
)

proto_library(
    name = "tracking_feature_proto_lib",
    srcs = ["tracking_feature.proto"],
)

cc_library(
    name = "external_feature_extractor",
    srcs = ["external_feature_extractor.cc"],
    hdrs = ["external_feature_extractor.h"],
    deps = [
        ":tracking_feature_proto",
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
        ":tracking_feature_proto",
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
        ":tracking_feature_proto",
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

py_proto(
    name = "tracking_feature_pb2",
    src = "tracking_feature.proto",
)

cpplint()
