load("@rules_proto//proto:defs.bzl", "proto_library")
load(
    "@com_github_grpc_grpc//bazel:python_rules.bzl",
    "py_proto_library",
)

package(default_visibility = ["//visibility:public"])

py_proto_library(
    name = "extractor_config_pb2",
    deps = [":extractor_config_proto_lib"],
)

proto_library(
    name = "extractor_config_proto_lib",
    srcs = ["extractor_config.proto"],
)
