load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load(
    "@com_github_grpc_grpc//bazel:python_rules.bzl",
    "py_proto_library",
)

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "udp_bridge_remote_info_proto",
    deps = [
        ":udp_bridge_remote_info_proto_lib",
    ],
)

proto_library(
    name = "udp_bridge_remote_info_proto_lib",
    srcs = ["udp_bridge_remote_info.proto"],
)

py_proto_library(
    name = "udp_bridge_remote_info_pb2",
    deps = [
        ":udp_bridge_remote_info_proto_lib",
    ],
)
