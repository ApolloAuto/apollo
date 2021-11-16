load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "monitor_log",
    deps = [
        ":monitor_log_buffer",
        ":monitor_logger",
        "//cyber/common:log",
    ],
)

cc_library(
    name = "monitor_logger",
    srcs = ["monitor_logger.cc"],
    hdrs = ["monitor_logger.h"],
    deps = [
        "//cyber",
        "//modules/common/monitor_log/proto:monitor_log_cc_proto",
        "//modules/common/util:message_util",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "monitor_log_buffer",
    srcs = ["monitor_log_buffer.cc"],
    hdrs = ["monitor_log_buffer.h"],
    deps = [
        ":monitor_logger",
        "//modules/common/monitor_log/proto:monitor_log_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "monitor_log_buffer_test",
    size = "small",
    srcs = ["monitor_log_buffer_test.cc"],
    deps = [
        ":monitor_log",
        "@com_google_googletest//:gtest_main",
    ],
)

# cc_test(
#     name = "monitor_logger_test",
#     size = "small",
#     srcs = ["monitor_logger_test.cc"],
#     deps = [
#         ":monitor_log",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cpplint()
