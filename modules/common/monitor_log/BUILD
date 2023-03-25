load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    library_dest = "common/lib",
    data_dest = "common",
    runtime_dest = "common/bin",
    targets = [
        ":libmonitor_log.so",
    ],
    visibility = ["//visibility:public"],
)


cc_binary(
    name = "libmonitor_log.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":monitor_log_buffer",
        ":monitor_logger",
	    "//modules/common_msgs/monitor_msgs:monitor_log_cc_proto",
    ],
)

cc_library(
    name = "monitor_log",
    srcs = ["libmonitor_log.so"],
    hdrs = [
        "monitor_log_buffer.h",
        "monitor_logger.h",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/monitor_msgs:monitor_log_cc_proto",
        "//modules/common/util:util_tool",
        "@com_google_googletest//:gtest",
        "@com_google_absl//:absl",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "monitor_logger",
    srcs = ["monitor_logger.cc"],
    hdrs = ["monitor_logger.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common_msgs/monitor_msgs:monitor_log_cc_proto",
        "//modules/common/util:util_tool",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "monitor_log_buffer",
    srcs = ["monitor_log_buffer.cc"],
    hdrs = ["monitor_log_buffer.h"],
    alwayslink = True,
    deps = [
        ":monitor_logger",
        "//cyber",
        "//modules/common_msgs/monitor_msgs:monitor_log_cc_proto",
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
