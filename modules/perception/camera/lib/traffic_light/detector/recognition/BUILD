load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "classify",
    srcs = ["classify.cc"],
    hdrs = ["classify.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/traffic_light/detector/recognition/proto:recognition_cc_proto",
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

cc_library(
    name = "recognition",
    srcs = ["recognition.cc"],
    hdrs = ["recognition.h"],
    deps = [
        ":classify",
        "//cyber/common:file",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_lib",
    ],
    alwayslink = True,
)

cpplint()
