load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "readme",
    srcs = [
        "README.md",
    ],
)

apollo_py_binary(
    name = "gmapviewer",
    srcs = ["gmapviewer.py"],
    deps = [
        "//modules/common_msgs/map_msgs:map_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

apollo_py_binary(
    name = "hdmapviewer",
    srcs = ["hdmapviewer.py"],
    deps = [
        "//modules/common_msgs/map_msgs:map_py_pb2",
        "//modules/tools/common:proto_utils",
    ],
)

# install(
#     name = "install",
#     data = [":readme"],
#     data_dest = "tools/mapviewers",
#     py_dest = "tools/mapviewers",
#     targets = [
#         "gmapviewer",
#         "hdmapviewer",
#     ]
# )

apollo_package()