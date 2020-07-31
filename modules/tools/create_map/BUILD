load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "convert_map_txt2bin",
    srcs = ["convert_map_txt2bin.py"],
    deps = [
        "//modules/map/proto:map_py_pb2",
    ],
)
