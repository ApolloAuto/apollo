load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interaction_predictor",
    srcs = ["interaction_predictor.cc"],
    hdrs = ["interaction_predictor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/common/adapters/proto:adapter_config_proto",
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/predictor/sequence:sequence_predictor",
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
