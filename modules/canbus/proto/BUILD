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
        "chassis.proto",
        "chassis_detail.proto",
        "ge3.proto",
        "lexus.proto",
        "transit.proto",
        "vehicle_parameter.proto",
    ],
    deps = [
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
        "//modules/drivers/canbus/proto:canbus_proto_lib",
    ],
)
