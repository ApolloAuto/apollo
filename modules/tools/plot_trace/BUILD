load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "plot_planning_result",
    srcs = ["plot_planning_result.py"],
    deps = [
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
    ],
)

py_binary(
    name = "plot_trace",
    srcs = ["plot_trace.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
    ],
)
