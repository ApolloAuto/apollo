load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "path_extract",
    srcs = ["path_extract.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/localization/proto:localization_py_pb2",
    ],
)

py_binary(
    name = "path_plot",
    srcs = ["path_plot.py"],
)

py_binary(
    name = "path_process",
    srcs = ["path_process.py"],
)

py_binary(
    name = "plot_gps_path",
    srcs = ["plot_gps_path.py"],
)
