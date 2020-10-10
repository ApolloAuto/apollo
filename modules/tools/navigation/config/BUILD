load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "navi_config",
    srcs = ["navi_config.py"],
    data = ["default.ini"],
    deps = [
        "//modules/dreamview/proto:hmi_config_py_pb2",
        "//modules/planning/proto:planning_config_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)
