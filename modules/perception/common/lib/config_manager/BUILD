load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "config_manager",
    srcs = ["config_manager.cc"],
    hdrs = ["config_manager.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm/io:io_util",
        "//modules/perception/common/lib/thread",
        "//modules/perception/common/proto:perception_config_schema_cc_proto",
        "@eigen",
    ],
)

apollo_package()
cpplint()
