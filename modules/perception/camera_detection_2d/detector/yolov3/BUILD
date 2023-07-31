load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cc_library(
    name = "postprocess",
    srcs = ["postprocess.cc"],
    hdrs = ["postprocess.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera_detection_2d/detector/yolov3/proto:yolo_model_param_cc_proto",
        "//modules/perception/common/base",
    ],
)

cc_library(
    name = "yolov3_obstacle_detector",
    srcs = ["yolov3_obstacle_detector.cc"],
    hdrs = ["yolov3_obstacle_detector.h"],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        ":postprocess",
        "//cyber",
        "//modules/perception/camera_detection_2d/interface:base_obstacle_detector",
        "//modules/perception/common:util",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
        "@opencv//:imgproc",
        "@local_config_cuda//cuda:cudart",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
