load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "resize_and_normalize",
    srcs = ["resize_and_normalize.cc"],
    hdrs = ["resize_and_normalize.h"],
    deps = [
       "//modules/perception/pipeline:data_frame",
       "//modules/perception/pipeline:plugin",
       "//modules/perception/pipeline/proto/plugin:resize_and_normalize_config_cc_proto",
    ],
)

cpplint()
