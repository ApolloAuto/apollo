load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "cyber_time",
    srcs = ["cyber_time.py"],
    data = [
        "//cyber/py_wrapper:_cyber_py3.so",
        "//cyber/py_wrapper:_cyber_time_py3.so",
    ],
)

py_library(
    name = "cyber_timer",
    srcs = ["cyber_timer.py"],
    data = [
        "//cyber/py_wrapper:_cyber_timer_py3.so",
    ],
)

py_library(
    name = "cyber",
    srcs = ["cyber.py"],
    data = [
        "//cyber/py_wrapper:_cyber_py3.so",
    ],
)

py_library(
    name = "parameter",
    srcs = ["parameter.py"],
    data = [
        "//cyber/py_wrapper:_cyber_parameter_py3.so",
    ],
)

py_library(
    name = "record",
    srcs = ["record.py"],
    data = [
        "//cyber/py_wrapper:_cyber_record_py3.so",
    ],
)
