load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "boost",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lboost_filesystem",
        "-lboost_program_options",
        "-lboost_regex",
        "-lboost_system",
        "-lboost_thread",
    ],
    strip_include_prefix = "include",
)