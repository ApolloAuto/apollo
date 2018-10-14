load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "logger_lib",
    srcs = glob([
        "async_logger.cpp",
        "log_file_object.cpp",
        "logger.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cybertron/common:common_lib",
    ],
)

cc_test(
    name = "logger_test",
    size = "small",
    srcs = [
        "logger_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cpplint()
