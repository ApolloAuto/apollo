load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test", "apollo_cc_binary")

apollo_cc_library(
    name = "kv_db",
    srcs = ["kv_db.cc"],
    hdrs = ["kv_db.h"],
    deps = [
        "//cyber",
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//:absl",
        "@sqlite3",
    ],
)

apollo_cc_test(
    name = "kv_db_test",
    size = "small",
    srcs = ["kv_db_test.cc"],
    deps = [
        ":kv_db",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_binary(
    name = "kv_db_tool",
    srcs = ["kv_db_tool.cc"],
    deps = [
        ":kv_db",
        "//cyber",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_package()

cpplint()
