load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "yaml-cpp",
    srcs = glob([
        "src/*.cpp",
    ]),
    hdrs = glob([
        "include/yaml-cpp/**/*.h",
    ]),
    includes = [
        "include",
        "src",
    ],
    deps = ["yaml-headers", "yaml-headers-without-prefix"],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "yaml-headers",
    hdrs = glob([
        "src/**/*.h",
    ]),
    include_prefix = "yaml-cpp",
    strip_include_prefix = "src",
)

cc_library(
    name = "yaml-headers-without-prefix",
    hdrs = glob([
        "src/**/*.h",
    ]),
    strip_include_prefix = "src",
)
