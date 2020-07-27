load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libspeaker_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":speaker_component_lib",
    ],
)

cc_library(
    name = "speaker_component_lib",
    srcs = ["speaker_component.cc"],
    hdrs = ["speaker_component.h"],
    copts = ['-DMODULE_NAME=\\"audio\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/audio:respeaker",
        "//modules/drivers/audio/proto:audio_cc_proto",
        "//modules/drivers/audio/proto:speaker_config_cc_proto",
    ],
)

cc_library(
    name = "respeaker",
    srcs = ["respeaker.cc"],
    hdrs = ["respeaker.h"],
    copts = ['-DMODULE_NAME=\\"audio\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/audio/proto:speaker_config_cc_proto",
        "@portaudio",
    ],
)

cpplint()