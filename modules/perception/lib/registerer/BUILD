load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "registerer",
    srcs = ["registerer.cc"],
    hdrs = ["registerer.h"],
    deps = [
        "//cyber",
    ],
)

cc_test(
    name = "registerer_test",
    size = "small",
    srcs = ["registerer_test.cc"],
    deps = [
        ":registerer",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
