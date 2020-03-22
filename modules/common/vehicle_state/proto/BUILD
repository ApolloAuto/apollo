load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "vehicle_state_proto",
    deps = [
        ":vehicle_state_proto_lib",
    ],
)

proto_library(
    name = "vehicle_state_proto_lib",
    srcs = ["vehicle_state.proto"],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/localization/proto:pose_proto_lib",
    ],
)

py_proto(
    name = "vehicle_state_pb2",
    src = "vehicle_state.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/localization/proto:pose_pb2",
    ],
)
