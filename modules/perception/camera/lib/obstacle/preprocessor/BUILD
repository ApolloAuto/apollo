load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_detection_preprocessor",
    srcs = ["camera_detection_preprocessor.cc"],
    hdrs = ["camera_detection_preprocessor.h"],
    deps = [
        "//modules/perception/pipeline:stage",
    ],
)

cpplint()
