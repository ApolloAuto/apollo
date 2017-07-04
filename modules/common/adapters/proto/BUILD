package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "adapter_config_proto",
    protos = [
        "adapter_config.proto",
    ],
)

py_proto_compile(
    name = "adapter_config_proto_pylib",
    protos = [
        "adapter_config.proto",
    ],
)
