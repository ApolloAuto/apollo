load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fem_pos_deviation_smoother",
    srcs = [
        "fem_pos_deviation_smoother.cc",
    ],
    hdrs = [
        "fem_pos_deviation_smoother.h",
    ],
    deps = [
        "//cyber/common:log",
        "@osqp",
    ],
)

cpplint()
