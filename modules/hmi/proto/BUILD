package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

py_proto_compile(
    name = "config_proto_pylib",
    protos = ["config.proto"],
)

cc_proto_library(
    name = "runtime_status_proto",
    protos = ["runtime_status.proto"],
)

py_proto_compile(
    name = "runtime_status_proto_pylib",
    protos = ["runtime_status.proto"],
)
