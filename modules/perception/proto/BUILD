package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "perception_proto",
    protos = [
        "perception_obstacle.proto",
        "traffic_light_detection.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto",
    ],
)

py_proto_compile(
    name = "perception_proto_pylib",
    protos = [
        "perception_obstacle.proto",
        "traffic_light_detection.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_pylib",
    ],
)
