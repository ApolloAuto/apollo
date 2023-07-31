load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "third_party_perception_test_base",
    srcs = ["third_party_perception_test_base.cc"],
    hdrs = ["third_party_perception_test_base.h"],
    deps = [
        "//modules/third_party_perception:third_party_perception_lib",
        "@com_google_googletest//:gtest",
    ],
)

apollo_package()
cpplint()
