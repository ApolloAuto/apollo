load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "yaml-cpp",
    srcs = glob([
        "src/*.cpp",
    ]),
    hdrs = glob([
        "include/yaml-cpp/*.h",
        "src/*.h",
    ]),
    includes = [
        "include",
        "src",
    ],
    visibility = ["//visibility:public"],
)
