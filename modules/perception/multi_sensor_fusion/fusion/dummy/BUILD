load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dummy_algorithms",
    srcs = ["dummy_algorithms.cc"],
    hdrs = ["dummy_algorithms.h"],
    deps = [
        "//cyber",
        "//modules/perception/multi_sensor_fusion/base",
        "//modules/perception/multi_sensor_fusion/interface",
        "//modules/perception/multi_sensor_fusion/interface:base_fusion_system",
        "@eigen",
    ],
    alwayslink = True,
)

cc_test(
    name = "dummy_algorithms_test",
    size = "small",
    srcs = ["dummy_algorithms_test.cc"],
    deps = [
        ":dummy_algorithms",
        "@com_google_googletest//:gtest_main",
        "//modules/perception/multi_sensor_fusion/interface:base_fusion_system",
    ],
)

apollo_package()
cpplint()
