load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "logger",
    srcs = [
        "logger.cc",
    ],
    hdrs = [
        "logger.h",
    ],
    deps = [
        "//cyber/base:macros",
        "//cyber/common",
        "//cyber/logger:log_file_object",
    ],
)

cc_test(
    name = "logger_test",
    size = "small",
    srcs = [
        "logger_test.cc",
    ],
    deps = [
        "//cyber",
        "@gtest//:main",
    ],
)

cc_library(
    name = "async_logger",
    srcs = [
        "async_logger.cc",
    ],
    hdrs = [
        "async_logger.h",
    ],
    deps = [
        "//cyber/base:macros",
        "//cyber/common",
        "//cyber/logger:log_file_object",
        "//cyber/scheduler:scheduler_factory",
    ],
)

cc_library(
    name = "log_file_object",
    srcs = [
        "log_file_object.cc",
    ],
    hdrs = [
        "log_file_object.h",
    ],
    deps = [
        "//cyber:binary",
        "//cyber/common:log",
        "//cyber/logger:logger_util",
    ],
)

cc_test(
    name = "log_file_object_test",
    size = "small",
    srcs = [
        "log_file_object_test.cc",
    ],
    deps = [
        "//cyber",
        "@gtest//:main",
    ],
)

cc_library(
    name = "logger_util",
    srcs = [
        "logger_util.cc",
    ],
    hdrs = [
        "logger_util.h",
    ],
    deps = [
        "//cyber/common:global_data",
    ],
)

cpplint()
