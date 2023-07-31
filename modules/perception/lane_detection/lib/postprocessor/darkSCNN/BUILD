load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "darkSCNN_lane_postprocessor",
    srcs = ["darkSCNN_lane_postprocessor.cc"],
    hdrs = ["darkSCNN_lane_postprocessor.h"],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/lib/interface:base_calibration_service",
        "//modules/perception/lane_detection/interface:base_lane_postprocessor",
        "//modules/perception/lane_detection/common:common_functions",
        "//modules/perception/lane_detection/proto:darkSCNN_cc_proto",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/lane_detection/proto:darkSCNN_postprocessor_cc_proto",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
