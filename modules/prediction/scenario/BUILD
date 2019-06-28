load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario_manager",
    srcs = ["scenario_manager.cc"],
    hdrs = ["scenario_manager.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/scenario/analyzer:scenario_analyzer",
        "//modules/prediction/scenario/feature_extractor",
        "//modules/prediction/scenario/prioritization:obstacles_prioritizer",
        "//modules/prediction/scenario/right_of_way",
    ],
)

cc_test(
    name = "scenario_manager_test",
    size = "small",
    srcs = ["scenario_manager_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":scenario_manager",
    ],
)

cpplint()
