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

py_binary(
    name = "talker",
    srcs = ["talker.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    name = "listener",
    srcs = ["listener.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    name = "parameter",
    srcs = ["parameter.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:parameter",
    ],
)

py_binary(
    name = "service",
    srcs = ["service.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    name = "client",
    srcs = ["client.py"],
    deps = [
        "//cyber/proto:unit_test_py_pb2",
        "//cyber/python/cyber_py3:cyber",
    ],
)

py_binary(
    name = "record_trans",
    srcs = ["record_trans.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
    ],
)

py_binary(
    name = "record_channel_info",
    srcs = ["record_channel_info.py"],
    deps = [
        "//cyber/proto:record_py_pb2",
        "//cyber/python/cyber_py3:cyber",
        "//cyber/python/cyber_py3:record",
    ],
)
