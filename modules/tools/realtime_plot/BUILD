load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "item",
    srcs = ["item.py"],
)

py_binary(
    name = "realtime_plot",
    srcs = ["realtime_plot.py"],
    deps = [
        ":stitem",
        ":xyitem",
        "//cyber/python/cyber_py3:cyber",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
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
