load("//tools/proto:proto.bzl", "apollo_py_binary", "apollo_py_library")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob(["example_data/*"]) + ["run.sh"],
)

apollo_py_binary(
    name = "plot_trajectory",
    srcs = ["plot_trajectory.py"],
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

apollo_py_library(
    name = "mkz_polygon",
    srcs = ["mkz_polygon.py"],
)

# install(
#     name = "install",
#     data_dest = "tools/planning/plot_trajectory",
#     py_dest = "tools/planning/plot_trajectory",
#     targets = [
#         ":main",
#     ]
# )

apollo_package()