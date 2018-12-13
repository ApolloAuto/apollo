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

cpplint()