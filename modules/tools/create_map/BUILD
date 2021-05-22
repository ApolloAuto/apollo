load("@rules_python//python:defs.bzl", "py_binary")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    targets = [
      ":convert_map_txt2bin",
    ],
    runtime_dest = "modules/tools/create_map",
)

py_binary(
    name = "convert_map_txt2bin",
    srcs = ["convert_map_txt2bin.py"],
    deps = [
        "//modules/map/proto:map_py_pb2",
    ],
)
