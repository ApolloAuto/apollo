load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "main",
    srcs = ["main.py"],
    data = [
        "//modules/tools/planning/plot_trajectory/example_data",
    ],
    deps = [
        ":mkz_polygon",
        "//modules/localization/proto:localization_py_pb2",
        "//modules/planning/proto:planning_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

py_library(
    name = "mkz_polygon",
    srcs = ["mkz_polygon.py"],
)
