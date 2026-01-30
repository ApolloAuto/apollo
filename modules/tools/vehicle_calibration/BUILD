load("//tools/proto:proto.bzl", "apollo_py_binary", "apollo_py_library")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

# install(
#     name = "install",
#     data = ["runtime_files"],
#     data_dest = "tools/vehicle_calibration",
#     py_dest = "tools/vehicle_calibration",
#     targets = [
#         ":data_collector",
#         ":plot_data",
#         ":plot_grid",
#         ":plot_results",
#         ":preprocess",
#         ":process_data",
#         ":result2pb",
#     ]
# )

apollo_py_binary(
    name = "data_collector",
    srcs = ["data_collector.py"],
    deps = [
        ":plot_data",
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_time",
        "//modules/common_msgs/chassis_msgs:chassis_py_pb2",
        "//modules/common_msgs/control_msgs:control_cmd_py_pb2",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)

apollo_py_binary(
    name = "plot_data",
    srcs = ["plot_data.py"],
    deps = [
        ":process",
    ],
)

apollo_py_binary(
    name = "plot_grid",
    srcs = ["plot_grid.py"],
)

apollo_py_binary(
    name = "plot_results",
    srcs = ["plot_results.py"],
)

apollo_py_binary(
    name = "preprocess",
    srcs = ["preprocess.py"],
    deps = [
        ":sanity_check",
        "//cyber/python/cyber_py3:cyber",
        "//modules/dreamview/proto:preprocess_table_py_pb2",
    ],
)

apollo_py_library(
    name = "process",
    srcs = ["process.py"],
)

apollo_py_binary(
    name = "process_data",
    srcs = ["process_data.py"],
    deps = [
        ":process",
    ],
)

apollo_py_binary(
    name = "result2pb",
    srcs = ["result2pb.py"],
    deps = [
        # "//modules/control/proto:calibration_table_py_pb2",
        # "//modules/control/proto:control_conf_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_py_library(
    name = "sanity_check",
    srcs = ["sanity_check.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/config_msgs:vehicle_config_py_pb2",
        "//modules/tools/common:file_utils",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_py_binary(
    name = "plot_calibration_table",
    srcs = ["plot_calibration_table.py"],
)

filegroup(
    name = "runtime_files",
    srcs = glob([
        "calibration_data_sample/**",
        "*.py",
        "*.sh",
    ]),
)

apollo_package()