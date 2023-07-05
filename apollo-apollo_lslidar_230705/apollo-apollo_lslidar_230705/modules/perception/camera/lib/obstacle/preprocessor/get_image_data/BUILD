load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "get_image_data",
    srcs = ["get_image_data.cc"],
    hdrs = ["get_image_data.h"],
    deps = [
        "//modules/perception/pipeline:data_frame",
        "//modules/perception/pipeline:plugin",
        "//modules/perception/pipeline/proto/plugin:get_image_data_config_cc_proto",
    ],
)

cpplint()
