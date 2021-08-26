load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "velometer",
    srcs = ["velometer.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/tools/common:logger",
        "//modules/tools/common:proto_utils",
        "//modules/common/proto:velometer_py_pb2",

    ],
)
