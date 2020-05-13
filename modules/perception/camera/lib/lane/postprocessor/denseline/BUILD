load("//tools:cpplint.bzl", "cpplint")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library")
load("@com_github_grpc_grpc//bazel:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "denseline_postprocessor_cc_proto",
    deps = [
        ":denseline_postprocessor_proto",
    ],
)

proto_library(
    name = "denseline_postprocessor_proto",
    srcs = ["denseline_postprocessor.proto"],
)

cc_library(
    name = "denseline_lane_postprocessor",
    srcs = ["denseline_lane_postprocessor.cc"],
    hdrs = ["denseline_lane_postprocessor.h"],
    deps = [
        ":denseline_postprocessor_cc_proto",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/lane/common:common_functions",
        "//modules/perception/camera/lib/lane/common:denseline_proto",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/utils",
    ],
)

py_proto_library(
    name = "denseline_postprocessor_py_pb2",
    deps = ["denseline_postprocessor_proto"],
)

cpplint()
