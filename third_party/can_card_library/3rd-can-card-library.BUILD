load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "hermes_can",
    hdrs = [
        "include/bcan.h",
    ],
    srcs = glob(["lib/libbcan.so"]),
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)