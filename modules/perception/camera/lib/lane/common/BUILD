load("//tools:cpplint.bzl", "cpplint")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library")
load("@com_github_grpc_grpc//bazel:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "denseline_cc_proto",
    deps = [
        ":denseline_proto",
    ],
)

proto_library(
    name = "denseline_proto",
    srcs = ["denseline.proto"],
)

cc_proto_library(
    name = "darkSCNN_cc_proto",
    deps = [
        ":darkSCNN_proto",
    ],
)

proto_library(
    name = "darkSCNN_proto",
    srcs = ["darkSCNN.proto"],
)

cc_library(
    name = "common_functions",
    srcs = ["common_functions.cc"],
    hdrs = ["common_functions.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "@eigen",
    ],
)

py_proto_library(
    name = "darkSCNN_py_pb2",
    deps = [":darkSCNN_proto"],
)

py_proto_library(
    name = "denseline_py_pb2",
    deps = [":denseline_proto"],
)

cpplint()
