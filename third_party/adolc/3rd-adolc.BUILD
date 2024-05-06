load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "adolc",
    includes = ["include"],
    linkopts = [
        "-ladolc",
    ],
    strip_include_prefix = "include",
)
