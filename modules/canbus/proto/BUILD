package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "canbus_proto",
    deps = [
        ":canbus_proto_lib",
    ],
)

proto_library(
    name = "canbus_proto_lib",
    deps = [
        ":canbus_conf_proto_lib",
        ":ch_proto_lib",
        ":chassis_detail_proto_lib",
        ":chassis_proto_lib",
        ":ge3_proto_lib",
        ":lexus_proto_lib",
        ":transit_proto_lib",
        ":vehicle_parameter_proto_lib",
        ":wey_proto_lib",
        ":zhongyun_proto_lib",
    ],
)

cc_proto_library(
    name = "ch_proto",
    deps = [
        ":ch_proto_lib",
    ],
)

proto_library(
    name = "ch_proto_lib",
    srcs = [
        "ch.proto",
    ],
)

cc_proto_library(
    name = "ge3_proto",
    deps = [
        ":ge3_proto_lib",
    ],
)

proto_library(
    name = "ge3_proto_lib",
    srcs = [
        "ge3.proto",
    ],
)

cc_proto_library(
    name = "lexus_proto",
    deps = [
        ":lexus_proto_lib",
    ],
)

proto_library(
    name = "lexus_proto_lib",
    srcs = [
        "lexus.proto",
    ],
)

cc_proto_library(
    name = "transit_proto",
    deps = [
        ":transit_proto_lib",
    ],
)

proto_library(
    name = "transit_proto_lib",
    srcs = [
        "transit.proto",
    ],
)

cc_proto_library(
    name = "wey_proto",
    deps = [
        ":wey_proto_lib",
    ],
)

proto_library(
    name = "wey_proto_lib",
    srcs = [
        "wey.proto",
    ],
)

cc_proto_library(
    name = "zhongyun_proto",
    deps = [
        ":zhongyun_proto_lib",
    ],
)

proto_library(
    name = "zhongyun_proto_lib",
    srcs = [
        "zhongyun.proto",
    ],
)

cc_proto_library(
    name = "chassis_proto",
    deps = [
        ":chassis_proto_lib",
    ],
)

proto_library(
    name = "chassis_proto_lib",
    srcs = [
        "chassis.proto",
    ],
    deps = [
        "//modules/common/proto:vehicle_signal_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

cc_proto_library(
    name = "vehicle_parameter_proto",
    deps = [
        ":vehicle_parameter_proto_lib",
    ],
)

proto_library(
    name = "vehicle_parameter_proto_lib",
    srcs = [
        "vehicle_parameter.proto",
    ],
    deps = [
        ":chassis_proto_lib",
        "//modules/common/configs/proto:vehicle_config_proto_lib",
    ],
)

cc_proto_library(
    name = "canbus_conf_proto",
    deps = [
        ":canbus_conf_proto_lib",
    ],
)

proto_library(
    name = "canbus_conf_proto_lib",
    srcs = [
        "canbus_conf.proto",
    ],
    deps = [
        ":vehicle_parameter_proto_lib",
        "//modules/drivers/canbus/proto:canbus_proto_lib",
    ],
)

cc_proto_library(
    name = "chassis_detail_proto",
    deps = [
        ":chassis_detail_proto_lib",
    ],
)

proto_library(
    name = "chassis_detail_proto_lib",
    srcs = [
        "chassis_detail.proto",
    ],
    deps = [
        ":ch_proto_lib",
        ":chassis_proto_lib",
        ":ge3_proto_lib",
        ":lexus_proto_lib",
        ":transit_proto_lib",
        ":wey_proto_lib",
        ":zhongyun_proto_lib",
    ],
)
