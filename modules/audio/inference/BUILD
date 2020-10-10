load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "moving_detection",
    srcs = ["moving_detection.cc"],
    hdrs = ["moving_detection.h"],
    deps = [
        "//modules/audio/proto:audio_cc_proto",
        "@fftw3",
    ],
)

cc_library(
    name = "direction_detection",
    srcs = ["direction_detection.cc"],
    hdrs = ["direction_detection.h"],
    deps = [
        "//cyber",
        "//modules/common/math:math_utils",
        "//modules/common/proto:geometry_cc_proto",
        "//third_party:libtorch",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
    ],
)

cc_library(
    name = "siren_detection",
    srcs = ["siren_detection.cc"],
    hdrs = ["siren_detection.h"],
    deps = [
        "//cyber/common:log",
        "//modules/audio/common:audio_gflags",
        "//third_party:libtorch",
    ],
)

# cc_test(
#     name = "siren_detection_test",
#     size = "small",
#     srcs = ["siren_detection_test.cc"],
#     deps = [
#         ":siren_detection",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cc_test(
    name = "moving_detection_test",
    size = "small",
    srcs = ["moving_detection_test.cc"],
    deps = [
        ":moving_detection",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
