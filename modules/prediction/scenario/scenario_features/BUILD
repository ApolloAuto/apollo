load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "scenario_features",
    srcs = ["cruise_scenario_features.cc"],
    hdrs = ["cruise_scenario_features.h", 
            "scenario_features.h"],
    deps = [
        "//framework:cybertron",
    ],
)

cc_test(
    name = "cruise_scenario_features_test",
    size = "small",
    srcs = [
        "cruise_scenario_features_test.cc",
    ],
    deps = [
        ":scenario_features",
        "@gtest//:main",
    ],
)

cpplint()
