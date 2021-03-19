load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "generator",
    srcs = ["generator.py"],
    deps = [
        "//modules/map/relative_map/proto:navigation_py_pb2",
    ],
)

py_binary(
    name = "viewer",
    srcs = ["viewer.py"],
    deps = [
        "//modules/map/relative_map/proto:navigation_py_pb2",
    ],
)
