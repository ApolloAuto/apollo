load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "path_extract",
    srcs = ["path_extract.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)

apollo_py_binary(
    name = "path_plot",
    srcs = ["path_plot.py"],
)

apollo_py_binary(
    name = "path_process",
    srcs = ["path_process.py"],
)

apollo_py_binary(
    name = "plot_gps_path",
    srcs = ["plot_gps_path.py"],
)


# install(
#     name = "install",
#     data_dest = "tools/navigation/driving_behavior",
#     py_dest = "tools/navigation/driving_behavior",
#     targets = [
#         "path_plot",
#         "path_process",
#         "plot_gps_path",
#         "path_extract",
#     ]
# )

apollo_package()