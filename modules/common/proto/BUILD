load("//tools:py_proto.bzl", "py_proto")

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

py_proto(
    name = "pnc_point_pb2",
    src = "pnc_point.proto",
)

py_proto(
    name = "drive_state_pb2",
    src = "drive_state.proto",
)

py_proto(
    name = "vehicle_signal_pb2",
    src = "vehicle_signal.proto",
)

py_proto(
    name = "drive_event_pb2",
    src = "drive_event.proto",
    deps = [
        "//modules/common/proto:header_pb2",
        "//modules/localization/proto:pose_pb2",
    ],
)

py_proto(
    name = "header_pb2",
    src = "header.proto",
    deps = [
        "//modules/common/proto:error_code_pb2",
    ],
)

py_proto(
    name = "geometry_pb2",
    src = "geometry.proto",
)

py_proto(
    name = "error_code_pb2",
    src = "error_code.proto",
)
