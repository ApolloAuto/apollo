load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load(
    "@com_github_grpc_grpc//bazel:python_rules.bzl",
    "py_proto_library",
)

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "error_code_proto",
    deps = [
        ":error_code_proto_lib",
    ],
)

proto_library(
    name = "error_code_proto_lib",
    srcs = ["error_code.proto"],
)

cc_proto_library(
    name = "header_proto",
    deps = [
        ":header_proto_lib",
    ],
)

proto_library(
    name = "header_proto_lib",
    srcs = ["header.proto"],
    deps = [
        ":error_code_proto_lib",
    ],
)

cc_proto_library(
    name = "vehicle_signal_proto",
    deps = [
        ":vehicle_signal_proto_lib",
    ],
)

proto_library(
    name = "vehicle_signal_proto_lib",
    srcs = ["vehicle_signal.proto"],
)

cc_proto_library(
    name = "geometry_proto",
    deps = [
        ":geometry_proto_lib",
    ],
)

proto_library(
    name = "geometry_proto_lib",
    srcs = ["geometry.proto"],
)

cc_proto_library(
    name = "pnc_point_proto",
    deps = [
        ":pnc_point_proto_lib",
    ],
)

proto_library(
    name = "pnc_point_proto_lib",
    srcs = ["pnc_point.proto"],
)

cc_proto_library(
    name = "drive_event_proto",
    deps = [
        ":drive_event_proto_lib",
    ],
)

proto_library(
    name = "drive_event_proto_lib",
    srcs = ["drive_event.proto"],
    deps = [
        ":header_proto_lib",
        "//modules/localization/proto:pose_proto_lib",
    ],
)

proto_library(
    name = "drive_state_proto_lib",
    srcs = ["drive_state.proto"],
)

cc_proto_library(
    name = "drive_state_proto",
    deps = [
        ":drive_state_proto_lib",
    ],
)

py_proto_library(
    name = "pnc_point_pb2",
    deps = [
        ":pnc_point_proto_lib",
    ],
)

py_proto_library(
    name = "drive_state_pb2",
    deps = [
        ":drive_state_proto_lib",
    ],
)

py_proto_library(
    name = "vehicle_signal_pb2",
    deps = [
        ":vehicle_signal_proto_lib",
    ],
)

py_proto_library(
    name = "drive_event_pb2",
    deps = [
        ":drive_event_proto_lib",
    ],
)

py_proto_library(
    name = "header_pb2",
    deps = [
        ":header_proto_lib",
    ],
)

py_proto_library(
    name = "geometry_pb2",
    deps = [
        ":geometry_proto_lib",
    ],
)

py_proto_library(
    name = "error_code_pb2",
    deps = [
        ":error_code_proto_lib",
    ],
)
