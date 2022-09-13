load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "get_image_data",
    srcs = ["get_image_data.cc"],
    hdrs = ["get_image_data.h"],
    deps = [
        "//modules/perception/pipeline:plugin",
    ],
)

cpplint()
