load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MICROPHONE_COPTS = ['-DMODULE_NAME=\\"microphone\\"']

cc_binary(
    name = "libmicrophone_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":microphone_component_lib",
    ],
)

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libmicrophone_component.so",
    ],
    deps = [
        "//cyber:install",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cc_library(
    name = "microphone_component_lib",
    srcs = ["microphone_component.cc"],
    hdrs = ["microphone_component.h"],
    copts = MICROPHONE_COPTS,
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
    copts = MICROPHONE_COPTS,
    deps = [
        "//cyber",
        "//modules/drivers/microphone/proto:microphone_config_cc_proto",
        "@portaudio",
    ],
)

cpplint()
