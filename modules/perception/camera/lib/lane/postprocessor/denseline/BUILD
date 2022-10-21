load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "denseline_lane_postprocessor",
    srcs = ["denseline_lane_postprocessor.cc"],
    hdrs = ["denseline_lane_postprocessor.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/lane/common:common_functions",
        "//modules/perception/lib/registerer",
        "//modules/perception/pipeline/proto/stage:denseline_cc_proto",
        "//modules/perception/pipeline/proto/stage:denseline_postprocessor_cc_proto",
        "//modules/perception/pipeline:stage",
    ],
)

cpplint()
