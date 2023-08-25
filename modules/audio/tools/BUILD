load("//tools:cpplint.bzl", "cpplint")
load("@rules_python//python:defs.bzl", "py_binary")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary")

package(default_visibility = ["//visibility:public"])

# py_binary(
#     name = "audiosaver",
#     srcs = ["audiosaver.py"],
#     deps = [
#         "//cyber/python/cyber_py3:cyber",
#         "//modules/common_msgs/basic_msgs:header_py_pb2",
#         "//modules/drivers/microphone/proto:audio_py_pb2",
#         "//modules/drivers/microphone/proto:microphone_config_py_pb2",
#     ],
#     tags = ["exclude"]
# )

apollo_cc_binary(
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
        "//modules/audio:apollo_audio",
        "//modules/audio/proto:audio_conf_cc_proto",
        "//modules/common_msgs/audio_msgs:audio_event_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "@boost",
    ],
)

apollo_package()
cpplint()
