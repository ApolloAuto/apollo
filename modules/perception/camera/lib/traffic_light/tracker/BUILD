load("//tools:cpplint.bzl", "cpplint")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library")
load("@com_github_grpc_grpc//bazel:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "semantic_cc_proto",
    deps = [
        ":semantic_proto",
    ],
)

proto_library(
    name = "semantic_proto",
    srcs = ["semantic.proto"],
)

cc_library(
    name = "semantic_decision",
    srcs = ["semantic_decision.cc"],
    hdrs = ["semantic_decision.h"],
    deps = [
        ":semantic_cc_proto",
        "//cyber/common:file",
        "//modules/perception/camera/lib/interface",
    ],
    alwayslink = True,
)

py_proto_library(
    name = "semantic_py_pb2",
    deps = ["semantic_proto"],
)

cpplint()
