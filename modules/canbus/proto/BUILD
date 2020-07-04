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
        "canbus_conf.proto",
        "ch.proto",
        "chassis.proto",
        "chassis_detail.proto",
        "ge3.proto",
        "lexus.proto",
        "transit.proto",
        "vehicle_parameter.proto",
        "wey.proto",
        "zhongyun.proto",
    ],
    deps = [
        "//modules/common/configs/proto:vehicle_config_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
        "//modules/drivers/canbus/proto:canbus_proto_lib",
    ],
)

py_proto(
    name = "zhongyun_pb2",
    src = "zhongyun.proto",
)

py_proto(
    name = "lexus_pb2",
    src = "lexus.proto",
)

py_proto(
    name = "chassis_detail_pb2",
    src = "chassis_detail.proto",
    deps = [
        "//modules/canbus/proto:ch_pb2",
        "//modules/canbus/proto:chassis_pb2",
        "//modules/canbus/proto:ge3_pb2",
        "//modules/canbus/proto:lexus_pb2",
        "//modules/canbus/proto:transit_pb2",
        "//modules/canbus/proto:wey_pb2",
        "//modules/canbus/proto:zhongyun_pb2",
        "//modules/common/configs/proto:vehicle_config_pb2",
    ],
)

py_proto(
    name = "wey_pb2",
    src = "wey.proto",
)

py_proto(
    name = "ch_pb2",
    src = "ch.proto",
)

py_proto(
    name = "transit_pb2",
    src = "transit.proto",
)

py_proto(
    name = "chassis_pb2",
    src = "chassis.proto",
    deps = [
        "//modules/common/configs/proto:vehicle_config_pb2",
        "//modules/common/proto:drive_state_pb2",
        "//modules/common/proto:geometry_pb2",
        "//modules/common/proto:header_pb2",
        "//modules/common/proto:vehicle_signal_pb2",
    ],
)

py_proto(
    name = "canbus_conf_pb2",
    src = "canbus_conf.proto",
    deps = [
        "//modules/canbus/proto:vehicle_parameter_pb2",
        "//modules/drivers/canbus/proto:can_card_parameter_pb2",
    ],
)

py_proto(
    name = "ge3_pb2",
    src = "ge3.proto",
)

py_proto(
    name = "vehicle_parameter_pb2",
    src = "vehicle_parameter.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/common/configs/proto:vehicle_config_pb2",
    ],
)
