load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_lmd_predictor_raw_base",
    deps = [
        "//modules/localization/lmd/predictor:localization_lmd_predictor",
    ],
)

cc_library(
    name = "localization_lmd_predictor_raw_imu",
    srcs = [
        "predictor_imu.cc",
    ],
    hdrs = [
        "predictor_imu.h",
    ],
    deps = [
        ":localization_lmd_predictor_raw_base",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/filters:digital_filter",
        "//modules/common/filters:digital_filter_coefficients",
        "//modules/drivers/gnss/proto:imu_proto",
        "//modules/localization/common:localization_common",
        "//modules/localization/proto:imu_proto",
    ],
)

cc_library(
    name = "localization_lmd_predictor_raw_gps",
    srcs = [
        "predictor_gps.cc",
    ],
    hdrs = [
        "predictor_gps.h",
    ],
    deps = [
        ":localization_lmd_predictor_raw_base",
        "//modules/common:log",
        "//modules/localization/proto:gps_proto",
    ],
)

cpplint()
