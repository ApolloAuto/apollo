load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "control_info",
    srcs = ["control_info.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/control/proto:control_cmd_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
    ],
)
