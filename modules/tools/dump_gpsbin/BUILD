load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "dump_gpsbin",
    srcs = ["dump_gpsbin.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/sensor_msgs:gnss_py_pb2",
    ],
)


# install(
#     name = "install",
#     py_dest = "tools/dump_gpsbin",
#     targets = [":dump_gpsbin"]
# )

apollo_package()