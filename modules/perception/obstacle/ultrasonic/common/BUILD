load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ultrasonic_kalman_filter",
    srcs = [
        "ultrasonic_kalman_filter.cc",
    ],
    hdrs = [
        "ultrasonic_kalman_filter.h",
    ],
    deps = [
        "//modules/perception/lib/base",
        "@eigen//:eigen",
    ],
)

cpplint()
