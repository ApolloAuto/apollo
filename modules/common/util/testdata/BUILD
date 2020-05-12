load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load(
    "@com_github_grpc_grpc//bazel:python_rules.bzl",
    "py_proto_library",
)
load("@rules_proto//proto:defs.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "simple_proto",
    deps = [
        ":simple_proto_lib",
    ],
)

proto_library(
    name = "simple_proto_lib",
    srcs = ["simple.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

py_proto_library(
    name = "simple_pb2",
    deps = [
        ":simple_proto_lib",
    ],
)
