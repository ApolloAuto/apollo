package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "prediction_proto",
    protos = [
        "prediction_obstacle.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto",
        "//modules/perception/proto:perception_proto",
    ],
)

py_proto_compile(
    name = "prediction_proto_pylib",
    protos = [
        "prediction_obstacle.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_pylib",
        "//modules/perception/proto:perception_proto_pylib",
    ],
)
