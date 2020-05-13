load("//tools:cpplint.bzl", "cpplint")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library")
load("@com_github_grpc_grpc//bazel:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "recognition_cc_proto",
    deps = [
        ":recognition_proto",
    ],
)

proto_library(
    name = "recognition_proto",
    srcs = ["recognition.proto"],
)

cc_library(
    name = "classify",
    srcs = ["classify.cc"],
    hdrs = ["classify.h"],
    deps = [
        ":recognition_cc_proto",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_factory_lib",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

cc_library(
    name = "recognition",
    srcs = ["recognition.cc"],
    hdrs = ["recognition.h"],
    deps = [
        ":classify",
        ":recognition_cc_proto",
        "//cyber/common:file",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_lib",
    ],
    alwayslink = True,
)

py_proto_library(
    name = "recognition_py_pb2",
    deps = ["recognition_proto"],
)

cpplint()
