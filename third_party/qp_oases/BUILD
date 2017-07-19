package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "qp_oases",
    srcs = [
        "lib/libqpOASES.so",
    ],
    hdrs = glob([
        "include/**",
    ]),
    includes = ["include"],
    include_prefix = "qpOASES",
)
