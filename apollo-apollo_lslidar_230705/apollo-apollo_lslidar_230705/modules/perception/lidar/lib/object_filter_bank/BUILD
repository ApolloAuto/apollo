load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "object_filter_bank",
    srcs = ["object_filter_bank.cc"],
    hdrs = ["object_filter_bank.h"],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/perception/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/lib/interface",
        "//modules/perception/pipeline:plugin_factory",
        "//modules/perception/pipeline:stage",
        "//modules/perception/pipeline/proto/stage:object_filter_bank_config_cc_proto",
        "//modules/perception/proto:perception_config_schema_cc_proto",
        "@eigen",
    ],
)

# todo(zero): need fix
# cc_test(
#     name = "object_filter_bank_test",
#     size = "small",
#     srcs = ["object_filter_bank_test.cc"],
#     deps = [
#         ":object_filter_bank",
#         "@com_github_gflags_gflags//:gflags",
#         "@com_google_googletest//:gtest_main",
#     ],
# )

cpplint()
