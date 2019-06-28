load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_sequence_predictor",
    srcs = ["lane_sequence_predictor.cc"],
    hdrs = ["lane_sequence_predictor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:validation_checker",
        "//modules/prediction/predictor/sequence:sequence_predictor",
    ],
)

cc_test(
    name = "lane_sequence_predictor_test",
    size = "small",
    srcs = [
        "lane_sequence_predictor_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/evaluator/vehicle:mlp_evaluator",
        "//modules/prediction/predictor/lane_sequence:lane_sequence_predictor",
    ],
)

cpplint()
