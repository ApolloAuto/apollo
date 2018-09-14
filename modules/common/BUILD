load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    deps = [
        ":log",
        "//framework:cybertron",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/common/util",
    ],
)

cc_library(
    name = "log",
    hdrs = [
        "log.h",
    ],
    deps = [
        "//framework:cybertron",
    ],
)

cpplint()
