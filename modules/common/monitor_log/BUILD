load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "monitor_log",
    srcs = [
        "monitor_log_buffer.cc",
        "monitor_logger.cc",
    ],
    hdrs = [
        "monitor_log_buffer.h",
        "monitor_logger.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor_log/proto:monitor_log_proto",
        "//modules/common/proto:common_proto",
        "//modules/common/time",
        "@gtest//:gtest",
    ],
)

cc_test(
    name = "monitor_log_buffer_test",
    size = "small",
    srcs = [
        "monitor_log_buffer_test.cc",
    ],
    deps = [
        ":monitor_log",
        "@gtest//:main",
    ],
)

cc_test(
    name = "monitor_logger_test",
    size = "small",
    srcs = [
        "monitor_logger_test.cc",
    ],
    deps = [
        ":monitor_log",
        "@gtest//:main",
    ],
)

cpplint()
