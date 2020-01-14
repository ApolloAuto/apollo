load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kv_db",
    srcs = ["kv_db.cc"],
    hdrs = ["kv_db.h"],
    deps = [
        "//cyber/common:log",
        "//modules/common/util",
        "//modules/common/util:future",
        "//third_party:sqlite3",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_test(
    name = "kv_db_test",
    size = "small",
    srcs = ["kv_db_test.cc"],
    deps = [
        ":kv_db",
        "@com_google_googletest//:gtest_main",
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
