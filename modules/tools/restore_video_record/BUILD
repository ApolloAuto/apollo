load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "restore_video_record",
    srcs = ["restore_video_record.py"],
    deps = [
        "//cyber/python/cyber_py3:record",
        "//modules/drivers/proto:sensor_image_py_pb2",
    ],
)
