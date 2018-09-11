load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "socketcan_checker_lib",
    srcs = [
        "socketcan_checker.cc",
        "socketcan_test.cc",
    ],
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/common/util:string_util",
        "//modules/monitor/hardware:hardware_checker",
    ],
)

cpplint()
