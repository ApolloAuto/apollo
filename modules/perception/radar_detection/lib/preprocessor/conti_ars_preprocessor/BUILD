load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "conti_ars_preprocessor_test",
    size = "small",
    srcs = ["conti_ars_preprocessor_test.cc"],
    deps = [
        ":conti_ars_preprocessor",
        "//modules/perception/common:perception_gflags",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "conti_ars_preprocessor",
    srcs = ["conti_ars_preprocessor.cc"],
    hdrs = ["conti_ars_preprocessor.h"],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common:util",
        "//modules/perception/common/base",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/radar_detection/common:types",
        "//modules/perception/radar_detection/interface:base_preprocessor",
        "//modules/perception/radar_detection/proto:preprocessor_cc_proto",
        "@eigen",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
