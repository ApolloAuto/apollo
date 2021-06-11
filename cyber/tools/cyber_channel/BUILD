load("@rules_python//python:defs.bzl", "py_binary")
load("//tools/install:install.bzl", "install_files")

package(
    default_visibility = ["//visibility:public"],
)

py_binary(
    name = "cyber_channel",
    srcs = ["cyber_channel.py"],
    deps = [
        "//cyber/proto:role_attributes_py_pb2",
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
    ],
)

install_files(
    name = "install",
    dest = "bin",
    files = [
        ":cyber_channel.py",
    ],
    rename = {
        "bin/cyber_channel.py": "cyber_channel",
    },
)
