load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fusion_tracker_gflags",
    srcs = ["fusion_tracker_gflags.cc"],
    hdrs = ["fusion_tracker_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "ft_config_manager",
    srcs = ["ft_config_manager.cc"],
    hdrs = ["ft_config_manager.h"],
    deps = [
        ":fusion_tracker_gflags",
        "//cyber",
        "//modules/v2x/proto:fusion_params_cc_proto",
    ],
)

cc_test(
    name = "ft_config_manager_test",
    size = "small",
    srcs = ["ft_config_manager_test.cc"],
    deps = [
        ":ft_config_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
