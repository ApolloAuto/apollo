package(default_visibility = ["//visibility:public"])

load("@org_pubref_rules_protobuf//cpp:rules.bzl", "cc_proto_library")
load("@org_pubref_rules_protobuf//python:rules.bzl", "py_proto_compile")

cc_proto_library(
    name = "control_proto",
    protos = [
        "calibration_table.proto",
        "control_cmd.proto",
        "control_conf.proto",
        "lat_controller_conf.proto",
        "lon_controller_conf.proto",
        "pad_msg.proto",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/proto:common_proto",
    ],
)

py_proto_compile(
    name = "control_proto_pylib",
    protos = [
        "calibration_table.proto",
        "control_cmd.proto",
        "control_conf.proto",
        "lat_controller_conf.proto",
        "lon_controller_conf.proto",
        "pad_msg.proto",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto_pylib",
        "//modules/common/proto:common_proto_pylib",
    ],
)
