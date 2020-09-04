load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "v2x_configs_lib",
    srcs = [
        "ft_config_manager.cc",
        "fusion_tracker_gflags.cc",
    ],
    hdrs = [
        "ft_config_manager.h",
        "fusion_tracker_gflags.h",
    ],
    copts = ['-DMODULE_NAME=\\"v2x_configs\\"'],
    deps = [
        "//cyber",
        "//modules/v2x/proto:fusion_params_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_test(
    name = "ft_config_manager_test",
    size = "small",
    srcs = ["ft_config_manager_test.cc"],
    deps = [
        ":v2x_configs_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
