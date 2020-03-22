load("//tools:py_proto.bzl", "py_proto")

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

py_proto(
    name = "pad_msg_pb2",
    src = "pad_msg.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "mpc_controller_conf_pb2",
    src = "mpc_controller_conf.proto",
    deps = [
        "//modules/control/proto:calibration_table_pb2",
        "//modules/control/proto:gain_scheduler_conf_pb2",
    ],
)

py_proto(
    name = "preprocessor_pb2",
    src = "preprocessor.proto",
    deps = [
        "//modules/common/proto:drive_state_pb2",
        "//modules/common/proto:header_pb2",
        "//modules/control/proto:input_debug_pb2",
        "//modules/control/proto:local_view_pb2",
    ],
)

py_proto(
    name = "lon_controller_conf_pb2",
    src = "lon_controller_conf.proto",
    deps = [
        "//modules/control/proto:calibration_table_pb2",
        "//modules/control/proto:leadlag_conf_pb2",
        "//modules/control/proto:pid_conf_pb2",
    ],
)

py_proto(
    name = "calibration_table_pb2",
    src = "calibration_table.proto",
)

py_proto(
    name = "input_debug_pb2",
    src = "input_debug.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "lat_controller_conf_pb2",
    src = "lat_controller_conf.proto",
    deps = [
        "//modules/control/proto:gain_scheduler_conf_pb2",
        "//modules/control/proto:leadlag_conf_pb2",
        "//modules/control/proto:mrac_conf_pb2",
    ],
)

py_proto(
    name = "control_conf_pb2",
    src = "control_conf.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/control/proto:lat_controller_conf_pb2",
        "//modules/control/proto:lon_controller_conf_pb2",
        "//modules/control/proto:mpc_controller_conf_pb2",
        "//modules/control/proto:pad_msg_pb2",
    ],
)

py_proto(
    name = "control_cmd_pb2",
    src = "control_cmd.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/common/proto:drive_state_pb2",
        "//modules/common/proto:header_pb2",
        "//modules/common/proto:pnc_point_pb2",
        "//modules/common/proto:vehicle_signal_pb2",
        "//modules/control/proto:input_debug_pb2",
        "//modules/control/proto:pad_msg_pb2",
    ],
)

py_proto(
    name = "leadlag_conf_pb2",
    src = "leadlag_conf.proto",
)

py_proto(
    name = "gain_scheduler_conf_pb2",
    src = "gain_scheduler_conf.proto",
)

py_proto(
    name = "control_common_conf_pb2",
    src = "control_common_conf.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/control/proto:pad_msg_pb2",
    ],
)

py_proto(
    name = "pid_conf_pb2",
    src = "pid_conf.proto",
)

py_proto(
    name = "local_view_pb2",
    src = "local_view.proto",
    deps = [
        "//modules/canbus/proto:chassis_pb2",
        "//modules/common/proto:header_pb2",
        "//modules/control/proto:pad_msg_pb2",
        "//modules/localization/proto:localization_pb2",
        "//modules/planning/proto:planning_pb2",
    ],
)

py_proto(
    name = "mrac_conf_pb2",
    src = "mrac_conf.proto",
)
