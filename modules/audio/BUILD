load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_cc_test", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_audio",
    srcs = [
        "common/audio_gflags.cc", 
        "common/audio_info.cc",
        "common/message_process.cc",
        "inference/direction_detection.cc",
        "inference/moving_detection.cc",
        "inference/siren_detection.cc",   
    ],
    hdrs = [
        "common/audio_gflags.h", 
        "common/audio_info.h",
        "common/message_process.h",
        "inference/direction_detection.h",
        "inference/moving_detection.h",
        "inference/siren_detection.h",   
    ],
    deps = [
        "@fftw3",
        "@libtorch_gpu",
        "@eigen",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@com_github_gflags_gflags//:gflags",
        "//cyber",
        "//modules/common/math",
        "//modules/drivers/microphone/proto:audio_cc_proto",
        "//modules/drivers/microphone/proto:microphone_config_cc_proto",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/audio_msgs:audio_cc_proto",
    ]
)

apollo_component(
    name = "libaudio_component.so",
    srcs = ["audio_component.cc"],
    hdrs = ["audio_component.h"],
    copts = [
        "-DMODULE_NAME=\\\"audio\\\"",
    ],
    deps = [
        "//cyber",
        ":apollo_audio",
        "//modules/audio/proto:audio_conf_cc_proto",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common/util:util_tool",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/**",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_cc_test(
    name = "moving_detection_test",
    size = "small",
    srcs = ["inference/moving_detection_test.cc"],
    deps = [
        ":apollo_audio",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "siren_detection_test",
    size = "small",
    srcs = ["inference/siren_detection_test.cc"],
    deps = [
        ":apollo_audio",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
