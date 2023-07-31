load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "detection",
    srcs = ["detection.cc"],
    hdrs = ["detection.h"],
    deps = [
        ":cropbox",
        ":select",
        "//cyber",
        "//modules/perception/common:util",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common:trafficlight_frame",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/inference:inference_factory",
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference:model_util",
        "//modules/perception/common/inference/utils:inference_resize_lib",
        "//modules/perception/common/inference/utils:inference_util_lib",
        "//modules/perception/traffic_light_detection/detection/proto:model_param_cc_proto",
        "//modules/perception/traffic_light_detection/interface:base_traffic_light_detector",
    ],
    alwayslink = True,
)

cc_library(
    name = "select",
    srcs = ["select.cc"],
    hdrs = ["select.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/algorithm/graph:hungarian_optimizer",
        "//modules/perception/common/algorithm/graph:secure_matrix",
    ],
)

cc_library(
    name = "cropbox",
    srcs = ["cropbox.cc"],
    hdrs = ["cropbox.h"],
    deps = [
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
    ],
)

apollo_package()
cpplint()
