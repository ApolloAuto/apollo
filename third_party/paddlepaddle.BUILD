package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "paddlepaddle",
    srcs = [
        "lib/libpaddle_fluid.so",
    ],
    hdrs = glob([
        "include/*.h",
    ]),
    includes = ["include"],
)
