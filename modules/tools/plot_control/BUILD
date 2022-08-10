load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "plot_control",
    srcs = ["plot_control.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/control_msgs:control_cmd_py_pb2",
    ],
)

py_binary(
    name = "realtime_test",
    srcs = ["realtime_test.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)
