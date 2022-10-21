load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "gmapviewer",
    srcs = ["gmapviewer.py"],
    deps = [
        "//modules/common_msgs/map_msgs:map_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

py_binary(
    name = "hdmapviewer",
    srcs = ["hdmapviewer.py"],
    deps = [
        "//modules/common_msgs/map_msgs:map_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)
