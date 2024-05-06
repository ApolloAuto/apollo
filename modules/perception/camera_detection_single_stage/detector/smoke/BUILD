load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

apollo_cc_library(
    name = "smoke_obstacle_detector",
    srcs = ["smoke_obstacle_detector.cc", "postprocess.cc"],
    hdrs = ["smoke_obstacle_detector.h", "postprocess.h"],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/interface:base_obstacle_detector",
        "//modules/perception/common/inference:apollo_perception_common_inference",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/camera_detection_single_stage/detector/smoke/proto:model_param_cc_proto",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "@opencv//:imgproc",
    ],
)

apollo_package()

cpplint()
