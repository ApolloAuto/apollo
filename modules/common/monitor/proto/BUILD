package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "monitor_proto",
    protos = [
        "monitor.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto",
    ],
)

py_proto_compile(
    name = "monitor_proto_pylib",
    protos = [
        "monitor.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_pylib",
    ],
)
