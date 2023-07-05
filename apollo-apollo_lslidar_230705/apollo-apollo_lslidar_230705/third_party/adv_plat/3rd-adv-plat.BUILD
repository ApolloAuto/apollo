load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "adv_plat",
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-ladv_trigger",
        "-ladv_bcan",
    ],
    strip_include_prefix = "include",
)
