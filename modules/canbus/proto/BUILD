package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "canbus_proto",
    protos = glob([
        "**/*.proto",
    ]),
    deps = [
        "//modules/common",
        "//modules/common/proto:common_proto",
    ],
)

py_proto_compile(
    name = "canbus_proto_pylib",
    protos = glob([
        "**/*.proto",
    ]),
    deps = [
        "//modules/common/proto:common_proto_pylib",
    ],
)
