load("@rules_cc//cc:defs.bzl", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "postprocess",
    srcs = ["postprocess.cc"],
    hdrs = ["postprocess.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/camera_detection_3d/detector/caddn/proto:model_param_cc_proto",
    ]
)

cc_library(
    name = "caddn_obstacle_detector",
    srcs = ["caddn_obstacle_detector.cc"],
    hdrs = ["caddn_obstacle_detector.h"],
    deps = [
        ":postprocess",
        "//cyber",
        "//modules/perception/common:util",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
        "//modules/perception/camera_detection_3d/interface:base_obstacle_detector",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common:util",
        "@eigen",
        "@opencv//:imgproc",
    ]
)

apollo_package()
cpplint()
