package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "hermes_can",
    srcs = [
        "lib/x86_64/libbcan.so",
    ],
    hdrs = [
        "include/bcan.h",
    ],
    include_prefix = "hermes_can",
)

cc_library(
    name = "hermes_can_aarch64",
    srcs = [
        "lib/aarch64/libbcan.so",
    ],
    hdrs = [
        "include/bcan.h",
    ],
    include_prefix = "hermes_can",
)
