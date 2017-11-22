load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "recurrent_runner",
    srcs = ["recurrent_runner.cc"],
    hdrs = ["recurrent_runner.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/time",
    ],
)

cc_library(
    name = "monitor_interface",
    srcs = ["monitor_interface.cc"],
    hdrs = ["monitor_interface.h"],
    deps = [
        ":recurrent_runner",
        "//modules/common/util",
        "//modules/common/util:map_util",
        "//modules/monitor/proto:monitor_conf_proto",
        "//modules/monitor/proto:system_status_proto",
    ],
)

cpplint()
