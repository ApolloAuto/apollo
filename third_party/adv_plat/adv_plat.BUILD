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
        ".",
    ],
    linkopts = [
        "-L/opt/apollo/pkgs/adv_plat/lib",
        "-ladv_trigger",
        "-ladv_bcan",
    ],
)
