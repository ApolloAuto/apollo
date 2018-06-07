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

cc_library(
    name = "extrin_ident",
    srcs = [
        "extrin_ident.cc",
    ],
    hdrs = [
        "extrin_ident.h",
    ],
    deps = [
        "@eigen//:eigen",
    ],
)

cpplint()
