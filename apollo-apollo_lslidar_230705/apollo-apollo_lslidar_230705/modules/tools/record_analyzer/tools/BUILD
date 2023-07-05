load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "dump_message",
    srcs = ["dump_message.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/control_msgs:control_cmd_py_pb2",
        "//modules/common_msgs/sensor_msgs:pointcloud_py_pb2",
        "//modules/common_msgs/perception_msgs:perception_obstacle_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
    ],
)

py_binary(
    name = "perception_obstacle_sender",
    srcs = ["perception_obstacle_sender.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/common_msgs/perception_msgs:perception_obstacle_py_pb2",
    ],
)

install(
    name = "install",
    py_dest = "tools/record_analyzer/tools",
    targets = [
        ":perception_obstacle_sender",
        ":dump_message",
    ]
)