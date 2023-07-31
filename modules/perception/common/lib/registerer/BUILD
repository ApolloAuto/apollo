load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "registerer",
    srcs = ["registerer.cc"],
    hdrs = ["registerer.h"],
    alwayslink = True,
    deps = [
        "//cyber",
    ],
)

cc_test(
    name = "registerer_test",
    size = "small",
    srcs = ["registerer_test.cc"],
    deps = [
        ":registerer",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
