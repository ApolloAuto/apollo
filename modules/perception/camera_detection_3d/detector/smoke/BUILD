load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cc_library(
    name = "postprocess",
    srcs = ["postprocess.cc"],
    hdrs = ["postprocess.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/camera_detection_3d/detector/smoke/proto:model_param_cc_proto",
    ],
)

cc_library(
    name = "smoke_obstacle_detector",
    srcs = ["smoke_obstacle_detector.cc"],
    hdrs = ["smoke_obstacle_detector.h"],
    alwayslink = True,
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        ":postprocess",
        "//cyber",
        "//modules/perception/common:util",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
        "//modules/perception/camera_detection_3d/interface:base_obstacle_detector",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common:util",
        "@opencv//:imgproc",
    ],
)

apollo_package()
cpplint()
