load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "denseline_lane_detector",
    srcs = ["denseline_lane_detector.cc"],
    hdrs = ["denseline_lane_detector.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
        "//modules/perception/lane_detection/interface:base_lane_detector",
        "//modules/perception/common/algorithm/sensor_manager",
        "//modules/perception/common/inference:inference_factory",
        "//modules/perception/common/inference/tensorrt:rt_net",
        "//modules/perception/common/inference/utils:inference_resize_lib",
        "//modules/perception/common/inference/utils:inference_util_lib",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/lane_detection/proto:denseline_cc_proto",
        "//modules/perception/common/inference/tensorrt:rt_legacy",
    ],
)

apollo_package()
cpplint()