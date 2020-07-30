load("@rules_python//python:defs.bzl", "py_binary", "py_library")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "audiosaver",
    srcs = ["audiosaver.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//modules/drivers/audio/proto:audio_py_pb2",
        "//modules/drivers/audio/proto:speaker_config_py_pb2",
        "//modules/common/proto:header_py_pb2",
    ],
)
