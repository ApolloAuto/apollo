load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "info_collector",
    srcs = ["info_collector.cc"],
    hdrs = ["info_collector.h"],
    deps = [
        "//modules/canbus/common:canbus_common",
        "//modules/common/adapters:adapter_manager",
        "//modules/control/common:control_gflags",
        "//modules/data/proto:recorder_conf_proto",
        "//modules/data/proto:task_proto",
    ],
)

cc_binary(
    name = "update_task_info",
    srcs = ["update_task_info.cc"],
    deps = [
        ":info_collector",
    ],
)

cc_test(
    name = "info_collector_test",
    srcs = ["info_collector_test.cc"],
    data = [
        "//modules/data/conf:recorder_conf",
    ],
    deps = [
        "@gtest//:main",
        ":info_collector",
    ],
)

cpplint()
