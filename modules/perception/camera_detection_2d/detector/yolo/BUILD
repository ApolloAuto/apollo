load("@rules_cc//cc:defs.bzl", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cc_library(
    name = "object_maintainer",
    srcs = ["object_maintainer.cc"],
    hdrs = ["object_maintainer.h"],
    deps = [
        "//modules/perception/common/base",
    ],
)

cuda_library(
    name = "postprocess_cuda",
    srcs = ["postprocess.cu"],
    hdrs = ["postprocess.h"],
    deps = [
        ":object_maintainer",
        "//modules/perception/common/base",
        "//modules/perception/common/proto:model_info_cc_proto",
        "//modules/perception/camera_detection_2d/detector/yolo/proto:model_param_cc_proto",
        "@local_config_cuda//cuda:cudart",
    ],
)

cc_library(
    name = "postprocess",
    srcs = [
        "postprocess.cc",
    ],
    hdrs = ["postprocess.h"],
    deps = [
        ":object_maintainer",
        ":postprocess_cuda",
        "//cyber",
        "@local_config_cuda//cuda:cudart",
    ],
)

cc_library(
    name = "yolo_obstacle_detector",
    srcs = ["yolo_obstacle_detector.cc"],
    hdrs = ["yolo_obstacle_detector.h"],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        ":postprocess",
        "//cyber",
        "//modules/perception/camera_detection_2d/interface:base_obstacle_detector",
        "//modules/perception/common:util",
        "//modules/perception/common/camera/common:util",
        "//modules/perception/common/inference/utils:inference_resize_cuda_lib",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
