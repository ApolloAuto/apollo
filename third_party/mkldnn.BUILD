load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "mkldnn",
    srcs = [
        "lib/libmkldnn.so.0",
    ],
    hdrs = glob([
        "include/*.h",
    ]),
)
