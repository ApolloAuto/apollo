load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "adv_plat",
    hdrs = [
        "adv_trigger.h",
        "bcan.h",
        "linux/bcan_defs.h",
        "linux/zynq_api.h",
    ],
    includes = [
        "include",
    ],
    linkopts = [
        "-ladv_trigger",
        "-ladv_bcan",
    ],
    strip_include_prefix = "include",
)
