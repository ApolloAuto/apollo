load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "viewer_raw",
    srcs = ["viewer_raw.py"],
)

py_binary(
    name = "viewer_smooth",
    srcs = ["viewer_smooth.py"],
)

py_binary(
    name = "record_extractor",
    srcs = ["record_extractor.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/common_msgs/localization_msgs:localization_py_pb2",
    ],
)