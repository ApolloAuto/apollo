load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "record",
    srcs = ["record.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:record",
        "//modules/common/proto:error_code_py_pb2",
        "//modules/common/proto:header_py_pb2",
        "//modules/common/util/testdata:simple_py_pb2",
    ],
)

py_binary(
    name = "time",
    srcs = ["time.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber_time",
    ],
)

py_binary(
    name = "timer",
    srcs = ["timer.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:cyber_timer",
    ],
)
