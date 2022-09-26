load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "absl",
    srcs = glob(["lib/*.so*"]),
    hdrs = glob(["include/*.h"]),
    includes = [
        "include",
    ],
)
