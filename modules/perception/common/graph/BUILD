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

cc_library(
    name = "graph_segmentor",
    srcs = [
        "graph_segmentor.cc",
    ],
    hdrs = [
        "graph_segmentor.h",
    ],
    deps = [
        "//framework:cybertron",
        ":disjoint_set",
    ],
)

cc_test(
    name = "graph_segmentor_test",
    size = "small",
    srcs = [
        "graph_segmentor_test.cc",
    ],
    deps = [
        ":graph_segmentor",
        "@gtest//:main",
    ],
)

cpplint()
