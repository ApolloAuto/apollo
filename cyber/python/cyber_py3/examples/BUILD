load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "record",
    srcs = ["record.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:record",
        "//modules/common/util/testdata:simple_py_pb2",
    ],
)
