load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "hermes_can",
    srcs = select(
        {
            "//tools/platform:x86_mode": [
                "lib_x86_64/libbcan.so",
            ],
            "//tools/platform:aarch64_mode": [
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
