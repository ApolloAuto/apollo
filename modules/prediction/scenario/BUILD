load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario_manager",
    srcs = ["scenario_manager.cc"],
    hdrs = ["scenario_manager.h"],
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/prediction/proto:prediction_conf_proto",
        "//modules/prediction/scenario/analyzer:scenario_analyzer",
        "//modules/prediction/scenario/feature_extractor:feature_extractor",
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
        "@gtest//:main",
    ],
)

cpplint()
