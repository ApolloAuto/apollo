load("@rules_python//python:defs.bzl", "py_test")

py_test(
    name = "test_record",
    timeout = "short",
    srcs = ["test_record.py"],
    deps = [
        "//cyber/proto:record_py_pb2",
        "//cyber/python/cyber_py3:record",
        "//modules/common/util/testdata:simple_py_pb2",
    ],
)
