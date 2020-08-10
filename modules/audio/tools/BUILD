load("@rules_python//python:defs.bzl", "py_binary")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "audiosaver",
    srcs = ["audiosaver.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/common/proto:header_py_pb2",
        "//modules/drivers/microphone/proto:audio_py_pb2",
        "//modules/drivers/microphone/proto:microphone_config_py_pb2",
    ],
)
