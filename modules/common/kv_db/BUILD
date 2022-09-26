load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    library_dest = "common/lib",
    data_dest = "common",
    runtime_dest = "common/bin",
    targets = [
        ":libkv_db.so",
        ":kv_db_tool",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libkv_db.so",
    srcs = [
        "kv_db.cc",
        "kv_db.h",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        "//cyber:cyber",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//:absl",
        "@sqlite3",
    ],
)

cc_library(
    name = "kv_db",
    srcs = ["libkv_db.so"],
    hdrs = ["kv_db.h"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//:absl",
        "@sqlite3",
    ],
    visibility = ["//visibility:public"],
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
        "//cyber",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
