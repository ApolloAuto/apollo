load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
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
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libaudio_component.so",
    ],
    deps = [
        "//cyber:install",
    ],
)

cpplint()
