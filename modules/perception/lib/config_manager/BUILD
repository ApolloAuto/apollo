load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "config_manager",
    srcs = ["config_manager.cc"],
    hdrs = ["config_manager.h"],
    deps = [
        "//cyber",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/io:io_util",
        "//modules/perception/lib/thread",
        "//modules/perception/proto:perception_config_schema_proto",
        "@eigen",
    ],
)

cc_test(
    name = "config_manager_test",
    size = "small",
    srcs = ["config_manager_test.cc"],
    data = [
        "//modules/perception:perception_testdata",
    ],
    deps = [
        ":config_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
