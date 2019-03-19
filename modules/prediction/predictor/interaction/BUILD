load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interaction_predictor",
    srcs = ["interaction_predictor.cc"],
    hdrs = ["interaction_predictor.h"],
    deps = [
        "//modules/prediction/predictor/sequence:sequence_predictor",
        "//modules/prediction/common:prediction_util",
    ],
)

cc_test(
    name = "interaction_predictor_test",
    size = "small",
    srcs = [
        "interaction_predictor_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/evaluator/vehicle:mlp_evaluator",
        "//modules/prediction/predictor/interaction:interaction_predictor",
    ],
)

cpplint()
