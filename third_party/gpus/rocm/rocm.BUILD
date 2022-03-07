load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "rocm_headers",
    hdrs = glob([
        "include/*.h",
    ]),
    includes = [
        "include",
    ],
)