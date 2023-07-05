load("@rules_python//python:defs.bzl", "py_binary")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "py_files",
    srcs = glob([
        "*.py",
    ]),
)

py_binary(
    name = "convert_map_txt2bin",
    srcs = ["convert_map_txt2bin.py"],
    deps = [
        "//modules/common_msgs/map_msgs:map_py_pb2",
    ],
)

install(
    name = "install",
    py_dest = "tools/create_map",
    targets = [":convert_map_txt2bin"]
)
