load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "sse2neon",
    includes = [
        "sse2neon.h",
    ],
    copts = ["-march=armv8-a+fp+simd+crypto+crc"],
)
