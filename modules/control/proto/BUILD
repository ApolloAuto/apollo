load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "control_proto",
    deps = [
        ":control_proto_lib",
    ],
)

proto_library(
    name = "control_proto_lib",
    srcs = [
        "control_cmd.proto",
        "control_conf.proto",
        "lat_controller_conf.proto",
        "leadlag_conf.proto",
        "lon_controller_conf.proto",
        "mrac_conf.proto",
        "pad_msg.proto",
        "pid_conf.proto",
    ],
    deps = [
        ":calibration_table_proto_lib",
        ":gain_scheduler_conf_proto_lib",
        ":mpc_controller_conf_proto_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
    ],
)

cc_proto_library(
    name = "mpc_controller_conf_proto",
    deps = [
        ":mpc_controller_conf_proto_lib",
    ],
)

proto_library(
    name = "mpc_controller_conf_proto_lib",
    srcs = [
        "mpc_controller_conf.proto",
    ],
    deps = [
        ":calibration_table_proto_lib",
        ":gain_scheduler_conf_proto_lib",
    ],
)

cc_proto_library(
    name = "gain_scheduler_conf_proto",
    deps = [
        ":gain_scheduler_conf_proto_lib",
    ],
)

proto_library(
    name = "gain_scheduler_conf_proto_lib",
    srcs = [
        "gain_scheduler_conf.proto",
    ],
)

cc_proto_library(
    name = "calibration_table_proto",
    deps = [
        ":calibration_table_proto_lib",
    ],
)

proto_library(
    name = "calibration_table_proto_lib",
    srcs = [
        "calibration_table.proto",
    ],
)

cpplint()
