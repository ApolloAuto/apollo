load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "item",
    srcs = ["item.py"],
)

py_binary(
    name = "realtime_plot",
    srcs = ["realtime_plot.py"],
    deps = [
        ":item",
        ":stitem",
        ":xyitem",
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

py_library(
    name = "stitem",
    srcs = ["stitem.py"],
)

py_library(
    name = "xyitem",
    srcs = ["xyitem.py"],
)

install(
    name = "install",
    py_dest = "tools/realtime_plot",
    targets = [
        ":realtime_plot",
    ]
)