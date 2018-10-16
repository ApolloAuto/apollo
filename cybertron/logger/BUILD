load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "logger",
    srcs = [
        "logger.cpp",
    ],
    hdrs = glob([
        "logger.h",
    ]),
    deps = [
        "//cybertron/common:common",
        "//cybertron/logger:log_file_object",
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

cc_library(
    name = "async_logger",
    srcs = [
        "async_logger.cpp",
    ],
    hdrs = glob([
        "async_logger.h",
    ]),
    deps = [
        "//cybertron/common:macros",
        "//cybertron/logger:logger",
        "//cybertron/logger:log_file_object",
    ]
)

cc_library(
    name = "log_file_object",
    srcs = [
        "log_file_object.cpp",
    ],
    hdrs = glob([
        "log_file_object.h",
    ]),
    deps = [
        "//cybertron/common:log",
        "//cybertron/logger:logger_util",
    ]
)

cc_library(
    name = "logger_util",
    hdrs = glob([
        "logger_util.h",
    ]),
)

cpplint()
