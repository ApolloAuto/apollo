load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "dump_gpsbin",
    srcs = ["dump_gpsbin.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
        "//modules/drivers/gnss/proto:gnss_py_pb2",
    ],
)
