load("//tools/proto:proto.bzl", "apollo_py_binary", "apollo_py_library")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "chassis_speed",
    srcs = ["chassis_speed.py"],
    deps = [
        ":record_reader",
    ],
)

apollo_py_binary(
    name = "imu_acc",
    srcs = ["imu_acc.py"],
    deps = [
        ":record_reader",
    ],
)

apollo_py_binary(
    name = "imu_angular_velocity",
    srcs = ["imu_angular_velocity.py"],
    deps = [
        ":record_reader",
    ],
)

apollo_py_binary(
    name = "imu_av_curvature",
    srcs = ["imu_av_curvature.py"],
    deps = [
        ":imu_angular_velocity",
        ":imu_speed",
        ":record_reader",
    ],
)

apollo_py_binary(
    name = "imu_speed",
    srcs = ["imu_speed.py"],
    deps = [
        ":record_reader",
    ],
)

apollo_py_binary(
    name = "imu_speed_acc",
    srcs = ["imu_speed_acc.py"],
    deps = [
        ":imu_speed",
        ":record_reader",
    ],
)

apollo_py_binary(
    name = "imu_speed_jerk",
    srcs = ["imu_speed_jerk.py"],
    deps = [
        ":imu_speed_acc",
        ":record_reader",
    ],
)

apollo_py_binary(
    name = "planning_speed",
    srcs = ["planning_speed.py"],
)

apollo_py_binary(
    name = "plot_acc_jerk",
    srcs = ["plot_acc_jerk.py"],
    deps = [
        ":imu_speed_acc",
        ":imu_speed_jerk",
        ":record_reader",
    ],
)

apollo_py_binary(
    name = "plot_chassis_acc",
    srcs = ["plot_chassis_acc.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/control_msgs:control_cmd_py_pb2",
    ],
)

apollo_py_binary(
    name = "plot_planning_acc",
    srcs = ["plot_planning_acc.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/control_msgs:control_cmd_py_pb2",
    ],
)

apollo_py_binary(
    name = "plot_planning_speed",
    srcs = ["plot_planning_speed.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/control_msgs:control_cmd_py_pb2",
    ],
)

apollo_py_binary(
    name = "plot_speed_jerk",
    srcs = ["plot_speed_jerk.py"],
    deps = [
        ":imu_speed",
        ":imu_speed_jerk",
        ":record_reader",
    ],
)

apollo_py_binary(
    name = "plot_speed_steering",
    srcs = ["plot_speed_steering.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
    ],
)

apollo_py_library(
    name = "record_reader",
    srcs = ["record_reader.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
    ],
)

apollo_py_binary(
    name = "speed_dsteering_data",
    srcs = ["speed_dsteering_data.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
    ],
)

# install(
#     name = "install",
#     py_dest = "tools/plot_planning",
#     targets = [
#         ":speed_dsteering_data",
#         ":plot_speed_steering",
#         ":plot_speed_jerk",
#         ":plot_planning_speed",
#         ":plot_planning_acc",
#         ":plot_chassis_acc",
#         ":plot_acc_jerk",
#         ":planning_speed",
#         ":imu_speed_jerk",
#         ":imu_speed_acc",
#         ":imu_speed",
#         ":imu_av_curvature",
#         ":imu_angular_velocity",
#         ":imu_acc",
#         ":chassis_speed",
#     ]
# )

apollo_package()