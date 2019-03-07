load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "finite_element1d_optimizer",
    srcs = [
        "finite_element1d_optimizer.cc",
    ],
    hdrs = [
        "finite_element1d_optimizer.h",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/planning/common:planning_gflags",
        "@osqp",
    ],
)

cc_test(
    name = "finite_element1d_optimizer_test",
    size = "small",
    srcs = [
        "finite_element1d_optimizer_test.cc",
    ],
    deps = [
        ":finite_element1d_optimizer",
        "//cyber/common:log",
        "@gtest//:main",
    ],
)

cpplint()
