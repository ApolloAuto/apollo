load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "openh264",
    includes = [
        "include",
    ],
    linkopts = [
        "-lopenh264",
    ],
    linkstatic = False,
    strip_include_prefix = "include",
)
