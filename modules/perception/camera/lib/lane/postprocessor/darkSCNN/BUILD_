load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "darkSCNN_lane_postprocessor",
    srcs = ["darkSCNN_lane_postprocessor.cc"],
    hdrs = ["darkSCNN_lane_postprocessor.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/lane/common:common_functions",
        "//modules/perception/camera/lib/lane/common/proto:darkSCNN_cc_proto",
        "//modules/perception/camera/lib/lane/postprocessor/darkSCNN/proto:darkSCNN_postprocessor_cc_proto",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/utils",
    ],
)

cpplint()
