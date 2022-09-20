load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "filter_bbox",
    srcs = ["filter_bbox.cc"],
    hdrs = ["filter_bbox.h"],
    deps = [
        "//modules/perception/pipeline:data_frame",
        "//modules/perception/pipeline:plugin",
        "//modules/perception/pipeline/proto/plugin:filter_bbox_config_cc_proto",
    ],
)

cpplint()
