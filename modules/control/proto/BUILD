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
        "control_conf.proto",
        "lat_controller_conf.proto",
        "lon_controller_conf.proto",
        "pid_conf.proto",
    ],
    deps = [
        ":calibration_table_proto_lib",
        ":control_command_proto_lib",
        ":gain_scheduler_conf_proto_lib",
        ":input_debug_lib",
        ":leadlag_conf_proto_lib",
        ":mpc_controller_conf_proto_lib",
        ":mrac_conf_proto_lib",
        ":pad_msg_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
    ],
)

# preprocessor_proto
cc_proto_library(
    name = "preprocessor_proto",
    deps = [
        ":preprocessor_proto_lib",
    ],
)

proto_library(
    name = "preprocessor_proto_lib",
    srcs = ["preprocessor.proto"],
    deps = [
        ":input_debug_lib",
        ":local_view_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

# local_view_proto
cc_proto_library(
    name = "local_view_proto",
    deps = [
        ":local_view_proto_lib",
    ],
)

proto_library(
    name = "local_view_proto_lib",
    srcs = ["local_view.proto"],
    deps = [
        ":pad_msg_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/localization/proto:localization_proto_lib",
        "//modules/planning/proto:planning_proto_lib",
    ],
)

# control command
cc_proto_library(
    name = "control_command_proto",
    deps = [
        ":control_command_proto_lib",
    ],
)

proto_library(
    name = "control_command_proto_lib",
    srcs = ["control_cmd.proto"],
    deps = [
        ":input_debug_lib",
        ":pad_msg_lib",
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
    ],
)

# mpc controller
cc_proto_library(
    name = "mpc_controller_conf_proto",
    deps = [
        ":mpc_controller_conf_proto_lib",
    ],
)

proto_library(
    name = "mpc_controller_conf_proto_lib",
    srcs = ["mpc_controller_conf.proto"],
    deps = [
        ":calibration_table_proto_lib",
        ":gain_scheduler_conf_proto_lib",
    ],
)

# gain_scheduler
cc_proto_library(
    name = "gain_scheduler_conf_proto",
    deps = [
        ":gain_scheduler_conf_proto_lib",
    ],
)

proto_library(
    name = "gain_scheduler_conf_proto_lib",
    srcs = ["gain_scheduler_conf.proto"],
)

# leadlag_conf
cc_proto_library(
    name = "leadlag_conf_proto",
    deps = [
        ":leadlag_conf_proto_lib",
    ],
)

proto_library(
    name = "leadlag_conf_proto_lib",
    srcs = ["leadlag_conf.proto"],
)

# mrac_conf
cc_proto_library(
    name = "mrac_conf_proto",
    deps = [
        ":mrac_conf_proto_lib",
    ],
)

proto_library(
    name = "mrac_conf_proto_lib",
    srcs = ["mrac_conf.proto"],
)

# calibration_table_proto
cc_proto_library(
    name = "calibration_table_proto",
    deps = [
        ":calibration_table_proto_lib",
    ],
)

proto_library(
    name = "calibration_table_proto_lib",
    srcs = ["calibration_table.proto"],
)

# control_common_conf_proto
cc_proto_library(
    name = "control_common_conf_proto",
    deps = [
        ":control_common_conf_proto_lib",
    ],
)

proto_library(
    name = "control_common_conf_proto_lib",
    srcs = ["control_common_conf.proto"],
    deps = [
        ":pad_msg_lib",
        "//modules/canbus/proto:canbus_proto_lib",
    ],
)

# pad_msg_proto
cc_proto_library(
    name = "pad_msg",
    deps = [
        ":pad_msg_lib",
    ],
)

proto_library(
    name = "pad_msg_lib",
    srcs = ["pad_msg.proto"],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:header_proto_lib",
    ],
)

# input_debug_proto
cc_proto_library(
    name = "input_debug",
    deps = [
        ":input_debug_lib",
    ],
)

proto_library(
    name = "input_debug_lib",
    srcs = ["input_debug.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()
