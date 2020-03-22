load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sequence_predictor",
    srcs = ["sequence_predictor.cc"],
    hdrs = ["sequence_predictor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/container:container_manager",
        "//modules/prediction/predictor",
    ],
)

cc_test(
    name = "sequence_predictor_test",
    size = "small",
    srcs = ["sequence_predictor_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/evaluator/vehicle:mlp_evaluator",
        "//modules/prediction/predictor/sequence:sequence_predictor",
    ],
)

cpplint()
