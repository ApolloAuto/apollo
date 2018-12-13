load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "db_helper",
    srcs = [
        "db_helper.cc",
    ],
    hdrs = [
        "db_helper.h",
    ],
    deps = [
        "@sqlite3",
        "//modules/common:log",
    ],
)

cc_library(
    name = "db_operator",
    srcs = [
        "db_operator.cc",
    ],
    hdrs = [
        "db_operator.h",
    ],
    deps = [
        ":db_helper",
    ],
)

cc_test(
    name = "db_operator_test",
    size = "small",
    srcs = [
        "db_operator_test.cc",
    ],
    deps = [
        ":db_operator",
        "@gtest//:main",
    ],
)

cpplint()