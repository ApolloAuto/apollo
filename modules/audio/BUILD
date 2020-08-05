load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
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
        "//modules/common/adapters:adapter_gflags",
        "//modules/audio/common:audio_info",
        "//modules/audio/proto:audio_cc_proto",
        "//modules/audio/proto:audio_conf_cc_proto",
        "//modules/drivers/microphone/proto:audio_cc_proto",
        "//modules/localization/proto:localization_cc_proto",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libaudio_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":audio_component_lib"],
)

cpplint()
