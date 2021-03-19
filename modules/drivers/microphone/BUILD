load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libmicrophone_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":microphone_component_lib",
    ],
)

cc_library(
    name = "microphone_component_lib",
    srcs = ["microphone_component.cc"],
    hdrs = ["microphone_component.h"],
    copts = ['-DMODULE_NAME=\\"microphone\\"'],
    deps = [
        "//cyber",
        "//modules/common/util:message_util",
        "//modules/drivers/microphone:respeaker",
        "//modules/drivers/microphone/proto:audio_cc_proto",
        "//modules/drivers/microphone/proto:microphone_config_cc_proto",
    ],
)

cc_library(
    name = "respeaker",
    srcs = ["respeaker.cc"],
    hdrs = ["respeaker.h"],
    copts = ['-DMODULE_NAME=\\"microphone\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/microphone/proto:microphone_config_cc_proto",
        "@portaudio",
    ],
)

cpplint()
