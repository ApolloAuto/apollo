load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario_features",
    srcs = ["scenario_features.cc"],
    hdrs = ["scenario_features.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/proto:scenario_proto",
    ],
)

cc_library(
    name = "cruise_scenario_features",
    srcs = ["cruise_scenario_features.cc"],
    hdrs = ["cruise_scenario_features.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        ":scenario_features",
        "//modules/prediction/common:environment_features",
        "//modules/prediction/common:prediction_map",
    ],
)

cc_library(
    name = "junction_scenario_features",
    srcs = ["junction_scenario_features.cc"],
    hdrs = ["junction_scenario_features.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        ":scenario_features",
        "//modules/prediction/common:environment_features",
    ],
)

cc_test(
    name = "cruise_scenario_features_test",
    size = "small",
    srcs = ["cruise_scenario_features_test.cc"],
    deps = [
        ":cruise_scenario_features",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
