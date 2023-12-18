load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_test", "apollo_package")

apollo_cc_library(
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
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/monitor_msgs:monitor_log_cc_proto",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest",
    ],
)

apollo_cc_test(
    name = "monitor_log_buffer_test",
    size = "small",
    srcs = ["monitor_log_buffer_test.cc"],
    deps = [
        ":monitor_log",
        "@com_google_googletest//:gtest_main",
    ],
)

# apollo_cc_test(
#     name = "monitor_logger_test",
#     size = "small",
#     srcs = ["monitor_logger_test.cc"],
#     deps = [
#         ":monitor_log",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

apollo_package()

cpplint()
