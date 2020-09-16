load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "v2x_fusion_component_lib",
    srcs = ["v2x_fusion_component.cc"],
    hdrs = ["v2x_fusion_component.h"],
    copts = [
        "-DMODULE_NAME=\\\"v2x_fusion\\\"",
    ],
    deps = [
        "//cyber/component",
        "//modules/common/adapters:adapter_gflags",
        "//modules/v2x/fusion/apps/common:trans_tools",
        "//modules/v2x/fusion/configs:ft_config_manager",
        "//modules/v2x/fusion/configs:fusion_tracker_gflags",
        "//modules/v2x/fusion/libs/fusion",
    ],
    alwayslink = True,
)

cc_binary(
    name = "libv2x_fusion_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":v2x_fusion_component_lib"],
)

cc_test(
    name = "v2x_fusion_component_test",
    size = "small",
    srcs = ["v2x_fusion_component_test.cc"],
    linkopts = [
        "-lgomp",
    ],
    deps = [
        ":v2x_fusion_component_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
