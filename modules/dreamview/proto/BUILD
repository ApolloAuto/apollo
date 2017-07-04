package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "simulation_world_proto",
    protos = ["simulation_world.proto"],
    deps = [
        "//modules/common/monitor/proto:monitor_proto",
        "//modules/decision/proto:decision_proto",
        "//modules/planning/proto:planning_proto",
    ],
)

py_proto_compile(
    name = "simulation_world_proto_pylib",
    protos = ["simulation_world.proto"],
    deps = [
        "//modules/common/monitor/proto:monitor_proto_pylib",
        "//modules/decision/proto:decision_proto_pylib",
        "//modules/planning/proto:planning_proto_pylib",
    ],
)
