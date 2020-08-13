load("@rules_cc//cc:defs.bzl", "cc_library")

# JSON for Modern C++
licenses(["notice"])  # 3-Clause BSD

exports_files(["LICENSE.MIT"])

cc_library(
    name = "json",
    hdrs = glob([
        "include/nlohmann/**/*.hpp",
    ]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    alwayslink = 1,
)

cc_library(
    name = "single_json",
    hdrs = glob(["single_include/**/*.hpp"]),
    strip_include_prefix = "single_include",
    visibility = ["//visibility:public"],
    alwayslink = 1,
)
