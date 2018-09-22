load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "disjoint_set",
    srcs = [
        "disjoint_set.cc",
    ],
    hdrs = [
        "disjoint_set.h",
    ],
)

cc_test(
    name = "disjoint_set_test",
    size = "small",
    srcs = [
        "disjoint_set_test.cc",
    ],
    deps = [
        ":disjoint_set",
        "@gtest//:main",
    ],
)

cpplint()
