load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "detection",
    srcs = ["detection.cc"],
    hdrs = ["detection.h"],
    deps = [
        ":cropbox",
        ":select",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/traffic_light/proto:detection_cc_proto",
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
    alwayslink = True,
)

cc_library(
    name = "select",
    srcs = ["select.cc"],
    hdrs = ["select.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/common/graph:hungarian_optimizer",
        "//modules/perception/common/graph:secure_matrix",
    ],
)

cc_library(
    name = "cropbox",
    srcs = ["cropbox.cc"],
    hdrs = ["cropbox.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/common",
    ],
)

cpplint()
