load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "canbus_proto",
    deps = [
        ":canbus_proto_lib",
    ],
)

proto_library(
    name = "canbus_proto_lib",
    srcs = [
        "can_card_parameter.proto",
        "sensor_canbus_conf.proto",
    ],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

py_proto(
    name = "can_card_parameter_pb2",
    src = "can_card_parameter.proto",
)

py_proto(
    name = "sensor_canbus_conf_pb2",
    src = "sensor_canbus_conf.proto",
    deps = [
        "//modules/drivers/canbus/proto:can_card_parameter_pb2",
    ],
)
