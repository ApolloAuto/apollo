load("//tools:cpplint.bzl", "cpplint")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "darkSCNN_postprocessor_cc_proto",
    deps = [
        ":darkSCNN_postprocessor_proto",
    ],
)

proto_library(
    name = "darkSCNN_postprocessor_proto",
    srcs = ["darkSCNN_postprocessor.proto"],
)

cc_library(
    name = "darkSCNN_lane_postprocessor",
    srcs = ["darkSCNN_lane_postprocessor.cc"],
    hdrs = ["darkSCNN_lane_postprocessor.h"],
    deps = [
        ":darkSCNN_postprocessor_cc_proto",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/lane/common:common_functions",
        "//modules/perception/camera/lib/lane/common:darkSCNN_proto",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/utils",
    ],
)

py_proto(
    name = "darkSCNN_postprocessor_pb2",
    deps = [":darkSCNN_postprocessor_proto"],
)

cpplint()
