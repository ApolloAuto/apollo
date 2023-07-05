load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "audio_component_lib",
    srcs = ["audio_component.cc"],
    hdrs = ["audio_component.h"],
    copts = [
        "-DMODULE_NAME=\\\"audio\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/audio/common:message_process",
        "//modules/audio/proto:audio_conf_cc_proto",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common/util:util_tool",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libaudio_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":audio_component_lib"],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

# Data->Models:
# modules/audio/data/torch_siren_detection_model.pt

install(
    name = "install",
    library_dest = "audio/lib",
    data_dest = "audio",
    data = [
        ":runtime_data",
        ":cyberfile.xml",
        ":audio.BUILD",
    ],
    targets = [
        ":libaudio_component.so",
    ],
    deps = [
        ":pb_hdrs",
        "//modules/audio/tools:install",
    ],
)

install(
    name = "pb_hdrs",
    data_dest = "audio/include",
    data = [
        "//modules/audio/proto:audio_conf_cc_proto",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_audio_src",
        ":install_audio_hdrs",
        ":install_audio_model",
        "//modules/audio/proto:py_pb_audio",
    ],
)

install_src_files(
    name = "install_audio_src",
    src_dir = ["."],
    dest = "audio/src",
    filter = "*",
)

install_src_files(
    name = "install_audio_model",
    src_dir = ["data"],
    dest = "audio/data",
    filter = "*"
)

install_src_files(
    name = "install_audio_hdrs",
    src_dir = ["."],
    dest = "audio/include",
    filter = "*.h",
)

cpplint()
