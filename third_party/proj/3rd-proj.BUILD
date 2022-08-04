load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "proj",
    includes = [
        "include",
    ],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lproj",
    ],
    linkstatic = False,
    strip_include_prefix = "include",
)
