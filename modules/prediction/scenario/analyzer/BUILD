load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario_analyzer",
    srcs = ["scenario_analyzer.cc"],
    hdrs = ["scenario_analyzer.h"],
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "//modules/common/proto:scenario_proto",
        "//modules/prediction/proto:scenario_feature_proto",
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
        "//modules/prediction/common:prediction_gflags",
        "@gtest//:main",
    ],
)

cpplint()
