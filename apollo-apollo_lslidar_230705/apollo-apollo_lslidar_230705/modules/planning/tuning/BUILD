load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "autotuning_raw_feature_generator",
    srcs = ["autotuning_raw_feature_generator.cc"],
    hdrs = ["autotuning_raw_feature_generator.h"],
    deps = [
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common:speed_limit",
        "//modules/planning/proto:auto_tuning_raw_feature_cc_proto",
    ],
)

cc_test(
    name = "autotuning_raw_feature_generator_test",
    size = "small",
    srcs = ["autotuning_raw_feature_generator_test.cc"],
    deps = [
        ":autotuning_raw_feature_generator",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "autotuning_feature_builder",
    hdrs = ["autotuning_feature_builder.h"],
    deps = [
        "//modules/common/status",
        "//modules/planning/proto:auto_tuning_model_input_cc_proto",
        "//modules/planning/proto:auto_tuning_raw_feature_cc_proto",
    ],
)

cc_library(
    name = "autotuning_mlp_net_model",
    srcs = ["autotuning_mlp_net_model.cc"],
    hdrs = ["autotuning_mlp_net_model.h"],
    deps = [
        "//cyber",
        "//modules/prediction/network:net_model",
    ],
    tags = ["exclude"]
)

cc_test(
    name = "autotuning_mlp_net_model_test",
    size = "small",
    srcs = ["autotuning_mlp_net_model_test.cc"],
    deps = [
        ":autotuning_mlp_net_model",
        "@com_google_googletest//:gtest_main",
    ],
    tags = ["exclude"]
)

cc_library(
    name = "autotuning_base_model",
    hdrs = ["autotuning_base_model.h"],
    deps = [
        ":autotuning_feature_builder",
        ":autotuning_mlp_net_model",
        "//modules/common/status",
        "//modules/planning/proto:auto_tuning_model_input_cc_proto",
    ],
    tags = ["exclude"]
)

cpplint()
