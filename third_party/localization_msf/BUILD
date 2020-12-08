load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_x86_64")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "localization_msf",
    srcs = if_x86_64([
        "x86_64/lib/liblocalization_msf.so",
    ]) + if_aarch64([
        "aarch64/lib/liblocalization_msf.so",
    ]),
    hdrs = if_x86_64(glob([
        "x86_64/include/*.h",
    ])) + if_aarch64(glob([
        "aarch64/include/*.h",
    ])),
    include_prefix = "localization_msf",
    linkopts = [
        "-lm",
    ],
    strip_include_prefix = select({
        "@platforms//cpu:x86_64": "x86_64/include",
        "@platforms//cpu:aarch64": "aarch64/include",
    }),
)
