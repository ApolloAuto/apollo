load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "static_info_reporter",
    srcs = ["static_info_reporter.cc"],
    hdrs = ["static_info_reporter.h"],
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/data/util:info_collector",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cpplint()
