load("@rules_python//python:defs.bzl", "py_binary")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "plot_planning_result",
    srcs = ["plot_planning_result.py"],
    deps = [
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
    ],
)

py_binary(
    name = "plot_trace",
    srcs = ["plot_trace.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)


install(
    name = "install",
    py_dest = "tools/plot_trace",
    targets = [
        ":plot_trace",
        ":plot_planning_result",
    ]
)