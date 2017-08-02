package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "routing_proto",
    protos = [
        "routing.proto",
        "topo_graph.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto",
        "//modules/map/proto:map_proto",
    ],
)

py_proto_compile(
    name = "routing_proto_pylib",
    protos = [
        "routing.proto",
        "topo_graph.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto_pylib",
        "//modules/map/proto:map_proto_pylib",
    ],
)
