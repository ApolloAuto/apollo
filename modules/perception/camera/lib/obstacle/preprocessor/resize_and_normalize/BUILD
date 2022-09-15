load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "resize_and_normalize",
    srcs = ["resize_and_normalize.cc"],
    hdrs = ["resize_and_normalize.h"],
    deps = [
        "//modules/perception/pipeline:plugin",
    ],
)

cpplint()
