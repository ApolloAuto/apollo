load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "hermes_can",
    srcs = select(
        {
            "@platforms//cpu:x86_64": [
                "lib_x86_64/libbcan.so",
            ],
            "@platforms//cpu:aarch64": [
                "lib_aarch64/libbcan.so",
            ],
        },
        no_match_error = "Please Build with a Linux aarch64 or x86_64 platform",
    ),
    hdrs = [
        "include/bcan.h",
    ],
    include_prefix = "hermes_can",
)
