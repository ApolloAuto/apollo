load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_lmd_predictor_output",
    srcs = [
        "predictor_output.cc",
        "predictor_print_error.cc",
    ],
    hdrs = [
        "predictor_output.h",
        "predictor_print_error.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/math",
        "//modules/common/proto:common_proto",
        "//modules/common/status",
        "//modules/localization/common:localization_common",
        "//modules/localization/lmd/common:localization_lmd_common",
        "//modules/localization/lmd/predictor:localization_lmd_predictor",
        "//modules/localization/proto:localization_proto",
        "@gtest",
    ],
)

cc_test(
    name = "localization_lmd_predictor_output_test",
    size = "small",
    srcs = [
        "predictor_output_test.cc"
    ],
    deps = [
        ":localization_lmd_predictor_output",
        "@gtest//:main",
    ],
)

cpplint()
