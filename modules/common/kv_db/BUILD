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
        "-lsqlite3",
    ],
    deps = [
        "//cyber/common:log",
        "//external:gflags",
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

cc_binary(
    name = "kv_db_tool",
    srcs = ["kv_db_tool.cc"],
    deps = [
        ":kv_db",
    ],
)

cpplint()
