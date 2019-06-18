load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "autotuning_speed_feature_builder",
    srcs = ["autotuning_speed_feature_builder.cc"],
    hdrs = ["autotuning_speed_feature_builder.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/proto:error_code_proto",
        "//modules/planning/tuning:autotuning_feature_builder",
    ],
)

cc_test(
    name = "autotuning_speed_feature_builder_test",
    size = "small",
    srcs = ["autotuning_speed_feature_builder_test.cc"],
    deps = [
        ":autotuning_speed_feature_builder",
        "@gtest//:main",
    ],
)

cc_library(
    name = "autotuning_speed_mlp_model",
    srcs = ["autotuning_speed_mlp_model.cc"],
    hdrs = ["autotuning_speed_mlp_model.h"],
    deps = [
        ":autotuning_speed_feature_builder",
        "//modules/planning/tuning:autotuning_base_model",
    ],
)

cc_test(
    name = "autotuning_speed_mlp_model_test",
    size = "small",
    srcs = ["autotuning_speed_mlp_model_test.cc"],
    deps = [
        ":autotuning_speed_mlp_model",
        "@gtest//:main",
    ],
)

cpplint()
