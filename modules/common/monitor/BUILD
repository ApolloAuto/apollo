load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "monitor",
    srcs = [
        "monitor.cc",
        "monitor_buffer.cc",
    ],
    hdrs = [
        "monitor.h",
        "monitor_buffer.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/monitor/proto:monitor_proto",
        "//modules/common/proto:common_proto",
        "//modules/common/time",
        "@glog//:glog",
    ],
)

cc_test(
    name = "monitor_buffer_test",
    size = "small",
    srcs = [
        "monitor_buffer_test.cc",
    ],
    deps = [
        ":monitor",
        "@gtest//:main",
    ],
)

cc_test(
    name = "monitor_test",
    size = "small",
    srcs = [
        "monitor_test.cc",
    ],
    deps = [
        ":monitor",
        "@gtest//:main",
    ],
)

cpplint()
