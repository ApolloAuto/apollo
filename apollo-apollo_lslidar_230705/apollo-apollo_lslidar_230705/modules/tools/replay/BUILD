load("@rules_python//python:defs.bzl", "py_binary")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "replay_file",
    srcs = ["replay_file.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/tools/common:message_manager",
        "//modules/tools/common:proto_utils",
    ],
)

install(
    name = "install",
    py_dest = "tools/replay",
    targets = [
        ":replay_file",
    ],
)