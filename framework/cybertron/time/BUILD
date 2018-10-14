load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "time_lib",
    srcs = glob([
        "duration.cpp",
        "rate.cpp",
        "time.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
          "//cybertron/common:common_lib",
    ],
)

cc_test(
    name = "duration_test",
    size = "small",
    srcs = [
        "duration_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "time_test",
    size = "small",
    srcs = [
        "time_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cpplint()
