load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_lmd_predictor_perception",
    srcs = [
        "lm_provider.cc",
        "lm_sampler.cc",
        "pc_map.cc",
        "pc_registrator.cc",
        "predictor_perception.cc",
    ],
    hdrs = [
        "lm_provider.h",
        "lm_sampler.h",
        "pc_map.h",
        "pc_registrator.h",
        "predictor_perception.h",
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
        "//modules/localization/proto:odometry_lane_marker_proto",
        "//modules/perception/proto:perception_proto",
        "@gtest",
    ],
)

cc_test(
    name = "localization_lmd_predictor_perception_test",
    size = "small",
    srcs = [
        "lm_provider_test.cc",
        "lm_sampler_test.cc",
        "pc_map_test.cc",
        "pc_registrator_test.cc",
        "predictor_perception_test.cc",
    ],
    data = ["//modules/localization:localization_testdata"],
    deps = [
        ":localization_lmd_predictor_perception",
        "@gtest//:main",
    ],
)

cpplint()
