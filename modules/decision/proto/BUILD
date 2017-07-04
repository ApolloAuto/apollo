package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "decision_proto",
    protos = [
        "decision.proto",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/proto:prediction_proto",
    ],
)

py_proto_compile(
    name = "decision_proto_pylib",
    protos = ["decision.proto"],
    deps = [
        "//modules/perception/proto:perception_proto_pylib",
        "//modules/prediction/proto:prediction_proto_pylib",
    ],
)
