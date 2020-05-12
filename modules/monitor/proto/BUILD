load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load(
    "@com_github_grpc_grpc//bazel:python_rules.bzl",
    "py_proto_library",
)

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "system_status_proto",
    deps = [
        ":system_status_proto_lib",
    ],
)

proto_library(
    name = "system_status_proto_lib",
    srcs = ["system_status.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

py_proto_library(
    name = "system_status_pb2",
    deps = [
        ":system_status_proto_lib",
    ],
)
