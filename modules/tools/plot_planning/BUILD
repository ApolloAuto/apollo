load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "chassis_speed",
    srcs = ["chassis_speed.py"],
    deps = [
        ":record_reader",
    ],
)

py_binary(
    name = "imu_acc",
    srcs = ["imu_acc.py"],
    deps = [
        ":record_reader",
    ],
)

py_binary(
    name = "imu_angular_velocity",
    srcs = ["imu_angular_velocity.py"],
    deps = [
        ":record_reader",
    ],
)

py_binary(
    name = "imu_av_curvature",
    srcs = ["imu_av_curvature.py"],
    deps = [
        ":imu_angular_velocity",
        ":imu_speed",
        ":record_reader",
    ],
)

py_binary(
    name = "imu_speed",
    srcs = ["imu_speed.py"],
    deps = [
        ":record_reader",
    ],
)

py_binary(
    name = "imu_speed_acc",
    srcs = ["imu_speed_acc.py"],
    deps = [
        ":imu_speed",
        ":record_reader",
    ],
)

py_binary(
    name = "imu_speed_jerk",
    srcs = ["imu_speed_jerk.py"],
    deps = [
        ":imu_speed_acc",
        ":record_reader",
    ],
)

py_binary(
    name = "planning_speed",
    srcs = ["planning_speed.py"],
)

py_binary(
    name = "plot_acc_jerk",
    srcs = ["plot_acc_jerk.py"],
    deps = [
        ":imu_speed_acc",
        ":imu_speed_jerk",
        ":record_reader",
    ],
)

py_binary(
    name = "plot_chassis_acc",
    srcs = ["plot_chassis_acc.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/control/proto:control_cmd_py_pb2",
    ],
)

py_binary(
    name = "plot_planning_acc",
    srcs = ["plot_planning_acc.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/control/proto:control_cmd_py_pb2",
    ],
)

py_binary(
    name = "plot_planning_speed",
    srcs = ["plot_planning_speed.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/control/proto:control_cmd_py_pb2",
    ],
)

py_binary(
    name = "plot_speed_jerk",
    srcs = ["plot_speed_jerk.py"],
    deps = [
        ":imu_speed",
        ":imu_speed_jerk",
        ":record_reader",
    ],
)

py_binary(
    name = "plot_speed_steering",
    srcs = ["plot_speed_steering.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
    ],
)

py_library(
    name = "record_reader",
    srcs = ["record_reader.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
    ],
)

py_binary(
    name = "speed_dsteering_data",
    srcs = ["speed_dsteering_data.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/canbus/proto:chassis_py_pb2",
    ],
)
