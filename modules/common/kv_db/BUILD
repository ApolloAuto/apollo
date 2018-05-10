load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kv_db",
    srcs = [
        "kv_db.cc",
    ],
    hdrs = [
        "kv_db.h",
    ],
    linkopts = [
        "-lleveldb",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/util",
    ],
)

cc_test(
    name = "kv_db_test",
    size = "small",
    srcs = [
        "kv_db_test.cc",
    ],
    deps = [
        ":kv_db",
        "@gtest//:main",
    ],
)

cpplint()
