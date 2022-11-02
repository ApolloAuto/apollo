load("@rules_python//python:defs.bzl", "py_binary", "py_library")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob(["example_data/*"]) + ["run.sh"],
)

py_binary(
    name = "main",
    srcs = ["main.py"],
    data = [
        "//modules/tools/planning/plot_trajectory/example_data",
    ],
    deps = [
        ":mkz_polygon",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
        "//modules/common_msgs/planning_msgs:planning_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

py_library(
    name = "mkz_polygon",
    srcs = ["mkz_polygon.py"],
)

install(
    name = "install",
    data_dest = "tools/planning/plot_trajectory",
    py_dest = "tools/planning/plot_trajectory",
    targets = [
        ":main",
    ]
)