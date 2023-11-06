load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

# filegroup(
#     name = "py_files",
#     srcs = glob([
#         "*.py",
#     ]),
# )

apollo_py_binary(
    name = "convert_map_txt2bin",
    srcs = ["convert_map_txt2bin.py"],
    deps = [
        "//modules/common_msgs/map_msgs:map_py_pb2",
    ],
)

# install(
#     name = "install",
#     py_dest = "tools/create_map",
#     targets = [":convert_map_txt2bin"]
# )

apollo_package()