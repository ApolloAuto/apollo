load("@rules_python//python:defs.bzl", "py_binary")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "readme",
    srcs = [
        "README.md",
        "default.ini",
    ],
)

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

install(
    name = "install",
    data = [":readme"],
    data_dest = "tools/navigation/config",
    py_dest = "tools/navigation/config",
    targets = [
        "navi_config",
    ]
)
