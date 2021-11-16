load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "endpoint",
    srcs = ["endpoint.cc"],
    hdrs = ["endpoint.h"],
    deps = [
        ":identity",
        "//cyber/common:global_data",
        "//cyber/proto:role_attributes_cc_proto",
    ],
)

cc_test(
    name = "endpoint_test",
    size = "small",
    srcs = ["endpoint_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "identity",
    srcs = ["identity.cc"],
    hdrs = ["identity.h"],
    deps = [
        "//cyber/common:util",
        "@uuid",
    ],
)

cc_test(
    name = "identity_test",
    size = "small",
    srcs = ["identity_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
