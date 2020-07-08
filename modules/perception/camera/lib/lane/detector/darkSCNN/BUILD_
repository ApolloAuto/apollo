load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "darkSCNN_lane_detector",
    srcs = ["darkSCNN_lane_detector.cc"],
    hdrs = ["darkSCNN_lane_detector.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/lane/common/proto:darkSCNN_cc_proto",
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference/caffe:caffe_net_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "//modules/perception/lib/registerer",
    ],
)

cpplint()
