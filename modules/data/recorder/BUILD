load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "info_collector",
    srcs = ["info_collector.cc"],
    hdrs = ["info_collector.h"],
    deps = [
        "//modules/canbus/common:canbus_common",
        "//modules/common/adapters:adapter_manager",
        "//modules/data/proto:task_proto",
    ],
)

cpplint()
