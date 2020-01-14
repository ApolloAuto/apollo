load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dummy_algorithms",
    srcs = ["dummy_algorithms.cc"],
    hdrs = ["dummy_algorithms.h"],
    deps = [
        "//cyber",
        "//modules/perception/fusion/base",
        "//modules/perception/fusion/lib/interface",
        "@eigen",
    ],
)

cc_test(
    name = "dummy_algorithms_test",
    size = "small",
    srcs = ["dummy_algorithms_test.cc"],
    deps = [
        ":dummy_algorithms",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
