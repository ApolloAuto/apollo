load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "audio_gflags",
    srcs = ["audio_gflags.cc"],
    hdrs = ["audio_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "audio_info",
    srcs = ["audio_info.cc"],
    hdrs = ["audio_info.h"],
    deps = [
        ":audio_gflags",
        "//modules/audio/proto:audio_cc_proto",
        "//modules/drivers/microphone/proto:audio_cc_proto",
        "//modules/drivers/microphone/proto:microphone_config_cc_proto",
    ],
)

cc_library(
    name = "message_process",
    srcs = ["message_process.cc"],
    hdrs = ["message_process.h"],
    deps = [
        ":audio_info",
        "//modules/audio/inference:direction_detection",
        "//modules/audio/inference:moving_detection",
        "//modules/audio/inference:siren_detection",
        "//modules/audio/proto:audio_cc_proto",
        "//modules/audio/proto:audio_conf_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/drivers/microphone/proto:audio_cc_proto",
    ],
)

cpplint()
