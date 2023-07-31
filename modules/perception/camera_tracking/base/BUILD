load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "frame_list",
    hdrs = ["frame_list.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera_tracking/common:camera_tracking_frame",
        "//modules/perception/common/inference/utils:inference_cuda_util_cuda_lib",
        "//modules/perception/common/inference/utils:inference_util_lib",
    ],
)

cc_library(
    name = "track_object",
    hdrs = ["track_object.h"],
    deps = [
        "//modules/perception/camera_tracking/base:frame_list",
    ],
)

cc_library(
    name = "target",
    srcs = ["target.cc"],
    hdrs = ["target.h"],
    deps = [
        "//modules/perception/camera_tracking/base:frame_list",
        ":track_object",
        "//modules/perception/common/algorithm/geometry:basic",
        "//modules/perception/common/camera/common",
        "//modules/perception/camera_tracking/common:kalman_filter",
        "//modules/perception/camera_tracking/proto:omt_cc_proto",
    ],
)

cc_library(
    name = "obstacle_reference",
    srcs = ["obstacle_reference.cc"],
    hdrs = ["obstacle_reference.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera_tracking/base:frame_list",
        "//modules/perception/camera_tracking/base:target",
        "//modules/perception/common/camera/common",
        "//modules/perception/camera_tracking/proto:omt_cc_proto",
        "//modules/perception/common/lib/interface:base_calibration_service",
    ],
)

cc_library(
    name = "base",
    deps = [
        ":frame_list",
        ":track_object",
        ":target",
        ":obstacle_reference",
    ],
)

apollo_package()
cpplint()