load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
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

cc_binary(
    name = "audio_offline_processing",
    srcs = ["audio_offline_processing.cc"],
    copts = [
        "-DMODULE_NAME=\\\"audio\\\"",
    ],
    linkopts = [
        "-lgomp",
    ],
    deps = [
        "//cyber",
        "//modules/audio/common:audio_gflags",
        "//modules/audio/common:message_process",
        "//modules/audio/proto:audio_event_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "@boost",
    ],
)

cpplint()
