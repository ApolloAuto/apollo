load("//tools:cpplint.bzl", "cpplint")

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

cpplint()
