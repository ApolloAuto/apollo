load("//tools:cpplint.bzl", "cpplint")
load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library")
load("@com_github_grpc_grpc//bazel:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "multicue_cc_proto",
    deps = [
        ":multicue_proto",
    ],
)

proto_library(
    name = "multicue_proto",
    srcs = ["multicue.proto"],
)

cc_library(
    name = "obj_mapper",
    srcs = ["obj_mapper.cc"],
    hdrs = ["obj_mapper.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera/common",
    ],
)

cc_library(
    name = "multicue_obstacle_transformer",
    srcs = ["multicue_obstacle_transformer.cc"],
    hdrs = ["multicue_obstacle_transformer.h"],
    deps = [
        ":multicue_cc_proto",
        ":obj_mapper",
        "//cyber",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
    ],
)

py_proto_library(
    name = "multicue_py_pb2",
    deps = [":multicue_proto"],
)

cpplint()
