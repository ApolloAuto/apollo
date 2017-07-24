package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "error_code_proto",
    protos = [
        "error_code.proto",
    ],
)

cc_proto_library(
    name = "header_proto",
    protos = [
        "header.proto",
    ],
    deps = [
        ":error_code_proto",
    ],
)

cc_proto_library(
    name = "vehicle_signal_proto",
    protos = [
        "vehicle_signal.proto",
    ],
)

cc_proto_library(
    name = "common_proto",
    protos = [
        "geometry.proto",
    ],
    deps = [
        ":error_code_proto",
        ":header_proto",
    ],
)

cc_proto_library(
    name = "gnss_status_proto",
    protos = [
        "gnss_status.proto",
    ],
    deps = [
        ":error_code_proto",
        ":header_proto",
    ],
)

cc_proto_library(
    name = "path_point_proto",
    protos = [
        "path_point.proto",
    ],
)

py_proto_compile(
    name = "common_proto_pylib",
    protos = [
        "error_code.proto",
        "geometry.proto",
        "header.proto",
        "path_point.proto",
        "vehicle_signal.proto",
    ],
)
