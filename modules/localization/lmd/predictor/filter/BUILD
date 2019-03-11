load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_lmd_predictor_filter_base",
    deps = [
        "//modules/common/math",
        "//modules/localization/lmd/predictor:localization_lmd_predictor",
    ],
)

cc_library(
    name = "localization_lmd_predictor_filtered_imu",
    srcs = [
        "predictor_filtered_imu.cc",
    ],
    hdrs = [
        "predictor_filtered_imu.h",
    ],
    deps = [
        ":localization_lmd_predictor_filter_base",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
    ],
)

cpplint()
