load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_inc_library(
    name = "hw_checker",
    hdrs = [
        "hw_checker.h",
        "annotations.h",
    ],
    deps = [
        "//modules/monitor/proto:hardware_status_proto",
    ]
)

cc_library(
    name = "log",
    srcs = ["log.cc"],
    hdrs = [
        "log.h",
        "common_defs.h",
    ],
)

cc_binary(
    name = "log_test",
    srcs = ["log_test.cc"],
    deps = [
        ":log",
    ],
)

cpplint()
