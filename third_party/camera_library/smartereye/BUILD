load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "smartereye",
    srcs = glob(["lib/*.so*"]),
    hdrs = glob(["include/*.h"]),
    copts = [
        "-Iinclude",
    ],
    includes = [
        "include",
    ],
    linkopts = [
        "-Llib",
    ],
    visibility = ["//visibility:public"],
)
