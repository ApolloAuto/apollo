load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "scenario_features",
    srcs = ["scenario_features.cc"],
    hdrs = ["scenario_features.h"],
    deps = [
        "//modules/prediction/proto:scenario_proto",
        "//framework:cybertron",
    ],
)

cc_library(
    name = "cruise_scenario_features",
    srcs = ["cruise_scenario_features.cc"],
    hdrs = ["cruise_scenario_features.h"],
    deps = [
        ":scenario_features",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/common:environment_features",
        "//cybertron",
    ],
)

cc_test(
    name = "cruise_scenario_features_test",
    size = "small",
    srcs = [
        "cruise_scenario_features_test.cc",
    ],
    deps = [
        ":cruise_scenario_features",
        "@gtest//:main",
    ],
)

cpplint()
