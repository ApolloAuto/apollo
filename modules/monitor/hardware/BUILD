load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_inc_library(
    name = "hardware_checker",
    hdrs = [
        "hardware_checker.h",
        "annotations.h",
    ],
    deps = [
        "//modules/monitor/proto:system_status_proto",
    ]
)

cc_inc_library(
    name = "hardware_check_utils",
    hdrs = [
        "hardware_check_utils.h",
    ],
    deps = [
        ":hardware_checker",
    ],
)

cpplint()
