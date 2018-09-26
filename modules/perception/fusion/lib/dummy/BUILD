load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dummy_algorithms",
    srcs = [
        "dummy_algorithms.cc",
    ],
    hdrs = [
        "dummy_algorithms.h",
    ],
    deps = [
        "//framework:cybertron",
        "//modules/perception/fusion/base",
        "//modules/perception/fusion/lib/interface",
        "@eigen",
    ],
)

cpplint()
