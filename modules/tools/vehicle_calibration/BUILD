load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "data_collector",
    srcs = ["data_collector.py"],
    deps = [
        ":plot_data",
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/canbus/proto:chassis_py_pb2",
        "//modules/control/proto:control_cmd_py_pb2",
        "//modules/localization/proto:localization_py_pb2",
    ],
)

py_binary(
    name = "plot_data",
    srcs = ["plot_data.py"],
    deps = [
        ":process",
    ],
)

py_binary(
    name = "plot_grid",
    srcs = ["plot_grid.py"],
)

py_binary(
    name = "plot_results",
    srcs = ["plot_results.py"],
)

py_library(
    name = "process",
    srcs = ["process.py"],
)

py_binary(
    name = "process_data",
    srcs = ["process_data.py"],
    deps = [
        ":process",
    ],
)

py_binary(
    name = "result2pb",
    srcs = ["result2pb.py"],
    deps = [
        "//modules/control/proto:calibration_table_py_pb2",
        "//modules/control/proto:control_conf_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)
