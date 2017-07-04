package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "vehicle_config_proto",
    protos = [
        "config_extrinsics.proto",
        "vehicle_config.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto",
    ],
)

py_proto_compile(
    name = "vehicle_config_proto_pylib",
    protos = [
        "config_extrinsics.proto",
        "vehicle_config.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_pylib",
    ],
)
