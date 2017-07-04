package(default_visibility = ["//visibility:public"])

cc_library(
    name = "time",
    hdrs = [
        "time.h",
    ],
    deps = [
        "//modules/common:macro",
    ],
)

cc_test(
    name = "time_test",
    size = "small",
    srcs = [
        "time_test.cpp",
    ],
    deps = [
        ":time",
        "@gtest//:main",
    ],
)
