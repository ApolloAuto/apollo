load("@rules_python//python:defs.bzl", "py_library")

package(default_visibility = ["//visibility:public"])

py_library(
    name = "record",
    srcs = ["record.py"],
    data = [
        "//cyber/py_wrapper:_cyber_record_py3.so",
    ],
)
