load("//tools:apollo_package.bzl", "apollo_package", "apollo_component", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MICROPHONE_COPTS = ['-DMODULE_NAME=\\"microphone\\"']

apollo_component(
    name = "libmicrophone_component.so",
    srcs = ["microphone_component.cc", "respeaker.cc"],
    hdrs = ["microphone_component.h", "respeaker.h"],
    copts = MICROPHONE_COPTS,
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/drivers/microphone/proto:audio_cc_proto",
        "//modules/drivers/microphone/proto:microphone_config_cc_proto",
        "@portaudio",
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

apollo_package()
cpplint()
