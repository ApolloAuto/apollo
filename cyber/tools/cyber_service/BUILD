load("@rules_python//python:defs.bzl", "py_binary")

package(
    default_visibility = ["//visibility:public"],
)

py_binary(
    name = "cyber_service",
    srcs = ["cyber_service.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
    ],
)
