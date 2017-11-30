load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_inc_library(
    name = "hardware_checker",
    hdrs = [
        "annotations.h",
        "hardware_checker.h",
    ],
    deps = [
        "//modules/monitor/proto:system_status_proto",
    ],
)

cpplint()
