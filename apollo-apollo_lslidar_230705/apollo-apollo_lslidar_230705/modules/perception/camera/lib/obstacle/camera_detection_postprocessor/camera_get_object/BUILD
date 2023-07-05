load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_get_object",
    srcs = ["camera_get_object.cc"],
    hdrs = ["camera_get_object.h"],
    deps = [
        "//modules/perception/pipeline:data_frame",
        "//modules/perception/pipeline:plugin",
        "//modules/perception/pipeline/proto/plugin:camera_get_object_config_cc_proto",
    ],
)

cpplint()
