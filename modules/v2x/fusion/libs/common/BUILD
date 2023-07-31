load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "v2x_object",
    hdrs = [
        "v2x_object.h",
    ],
    deps = [
        "//modules/perception/common/base:base_type",
        "//modules/perception/common/base:box",
        "//modules/perception/common/base:object",
        "@com_google_absl//:absl",
    ],
)

cc_test(
    name = "v2x_object_test",
    size = "small",
    srcs = ["v2x_object_test.cc"],
    deps = [
        ":v2x_object",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
