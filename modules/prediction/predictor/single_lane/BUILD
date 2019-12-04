load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "single_lane_predictor",
    srcs = ["single_lane_predictor.cc"],
    hdrs = ["single_lane_predictor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/predictor/sequence:sequence_predictor",
    ],
)

cc_test(
    name = "single_lane_predictor_test",
    size = "small",
    srcs = ["single_lane_predictor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/evaluator/vehicle:cost_evaluator",
        "//modules/prediction/predictor/single_lane:single_lane_predictor",
    ],
)

cpplint()
