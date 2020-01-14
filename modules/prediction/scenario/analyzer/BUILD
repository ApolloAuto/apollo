load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario_analyzer",
    srcs = ["scenario_analyzer.cc"],
    hdrs = ["scenario_analyzer.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/scenario/scenario_features:cruise_scenario_features",
        "//modules/prediction/scenario/scenario_features:junction_scenario_features",
    ],
)

cc_test(
    name = "scenario_analyzer_test",
    size = "small",
    srcs = ["scenario_analyzer_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":scenario_analyzer",
        "//modules/prediction/common:kml_map_based_test",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
