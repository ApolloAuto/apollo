package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "planning_proto",
    protos = [
        "planning.proto",
        "planning_internal.proto",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/proto:common_proto",
        "//modules/decision/proto:decision_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/proto:prediction_proto",
    ],
)

py_proto_compile(
    name = "planning_proto_pylib",
    protos = [
        "planning.proto",
        "planning_internal.proto",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto_pylib",
        "//modules/common/proto:common_proto_pylib",
        "//modules/decision/proto:decision_proto_pylib",
        "//modules/localization/proto:localization_proto_pylib",
        "//modules/perception/proto:perception_proto_pylib",
        "//modules/prediction/proto:prediction_proto_pylib",
    ],
)
