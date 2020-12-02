load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/platform:build_defs.bzl", "if_aarch64_mode", "if_x86_mode")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "localization_msf",
    srcs = if_x86_mode([
        "x86_64/lib/liblocalization_msf.so",
    ]) + if_aarch64_mode([
        "aarch64/lib/liblocalization_msf.so",
    ]),
    hdrs = if_x86_mode(glob([
        "x86_64/include/*.h",
    ])) + if_aarch64_mode(glob([
        "aarch64/include/*.h",
    ])),
    include_prefix = "localization_msf",
    linkopts = [
        "-lm",
    ],
    strip_include_prefix = select({
        "//tools/platform:x86_mode": "x86_64/include",
        "//tools/platform:aarch64_mode": "aarch64/include",
    }),
)
