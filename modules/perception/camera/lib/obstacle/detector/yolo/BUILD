load("@rules_cc//cc:defs.bzl", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "region_output_cuda",
    srcs = ["region_output.cu"],
    hdrs = ["region_output.h"],
    deps = [
        ":object_maintainer",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/proto:yolo_cc_proto",
        "@local_config_cuda//cuda:cudart",
    ],
)

cc_library(
    name = "region_output",
    srcs = [
        "region_output.cc",
        ":region_output_cuda",
    ],
    hdrs = ["region_output.h"],
    deps = [
        ":object_maintainer",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/proto:yolo_cc_proto",
        "@local_config_cuda//cuda:cudart",
    ],
)

cc_library(
    name = "yolo_obstacle_detector",
    srcs = ["yolo_obstacle_detector.cc"],
    hdrs = ["yolo_obstacle_detector.h"],
    deps = [
        ":region_output",
        "//cyber/common:file",
        "//modules/common/util:perf_util",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/feature_extractor/tfe:external_feature_extractor",
        "//modules/perception/camera/lib/feature_extractor/tfe:project_feature",
        "//modules/perception/camera/lib/feature_extractor/tfe:tracking_feat_extractor",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

cc_library(
    name = "object_maintainer",
    srcs = ["object_maintainer.cc"],
    hdrs = ["object_maintainer.h"],
    deps = [
        "//cyber/common:file",
        "//modules/perception/base",
    ],
)

cpplint()
