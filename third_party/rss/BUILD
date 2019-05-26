package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "rss",
    srcs = [
        "lib/librss_module.so",
        "lib/librss_support.so",
    ],
    hdrs = glob([
        "include/**/*.hpp",
    ]),
    includes = [
        "include",
    ],
)
