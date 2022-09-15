load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_detection_postprocessor",
    srcs = ["camera_detection_postprocessor.cc"],
    hdrs = ["camera_detection_postprocessor.h"],
    deps = [
        "//modules/perception/pipeline:stage",
    ],
)

cpplint()
