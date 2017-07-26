package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "prediction_conf_proto",
    protos = [
        "prediction_conf.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto",
        "//modules/perception/proto:perception_proto",
    ],
)

cc_proto_library(
    name = "prediction_proto",
    protos = [
        "prediction_obstacle.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto",
        "//modules/common/proto:path_point_proto",
        "//modules/perception/proto:perception_proto",
    ],
)

cc_proto_library(
    name = "lane_graph_proto",
    protos = [
        "lane_graph.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto",
    ],
)

cc_proto_library(
    name = "feature_proto",
    protos = [
        "feature.proto",
    ],
    deps = [
        "//modules/common/proto:common_proto",
        "//modules/prediction/proto:lane_graph_proto",
    ],
)

cc_proto_library(
    name = "fnn_model_base_proto",
    protos = [
        "fnn_model_base.proto",
    ],
    deps = [
    ],
)

cc_proto_library(
    name = "fnn_vehicle_model_proto",
    protos = [
        "fnn_vehicle_model.proto",
    ],
    deps = [
        "//modules/prediction/proto:fnn_model_base_proto",
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
