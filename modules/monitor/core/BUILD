load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sysmon_core",
    srcs = ["plugin_manager.cc"],
    hdrs = ["plugin_manager.h"],
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/monitor/sysmon/base:plugin_interface",
        "//third_party/json",
    ],
)

cc_library(
    name = "metric_plugin_for_test",
    srcs = ["metric_plugin_for_test.cc"],
    hdrs = [],
    deps = [
        "//modules/monitor/sysmon/base:metric_data_single_des_plugin",
    ],
)

cc_library(
    name = "datades_plugin_for_test",
    srcs = ["datades_plugin_for_test.cc"],
    hdrs = [],
    deps = [
        "//modules/monitor/sysmon/base:metric_data_single_des_plugin",
    ],
)

cc_test(
    name = "test_plugin_manager",
    size = "small",
    srcs = [
        "test_plugin_manager.cc",
    ],
    deps = [
        ":sysmon_core",
        "//modules/monitor/sysmon/base:metric_data_single_des_plugin",
        "@gtest//:main",
    ],
    linkopts= [ "-ldl" ],
)

cc_library(
    name = "sysmon_defs",
    srcs = [],
    hdrs = ["defs.h"],
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
    ]
)

cc_library(
    name = "periodic_task",
    srcs = ["periodic_task.cc"],
    hdrs = ["periodic_task.h"],
    deps = [
        ":sysmon_defs",
    ],
)

cc_test(
    name = "test_periodic_task",
    size = "small",
    srcs = [
        "test_periodic_task.cc",
    ],
    deps = [
        ":periodic_task",
        "@gtest//:main",
    ],
)

cc_library(
    name = "query_runner",
    srcs = [
        "query_runner.cc",
    ],
    hdrs = ["query_runner.h"],
    deps = [
        ":periodic_task",
        ":sysmon_defs",
    ],
)

cc_test(
    name = "test_query_runner",
    size = "small",
    srcs = [
        "test_query_runner.cc",
    ],
    deps = [
        ":query_runner",
        "@gtest//:main",
    ],
)

cpplint()
