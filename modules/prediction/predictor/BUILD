load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "predictor_manager",
    srcs = ["predictor_manager.cc"],
    hdrs = ["predictor_manager.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:prediction_thread_pool",
        "//modules/prediction/predictor/empty:empty_predictor",
        "//modules/prediction/predictor/extrapolation:extrapolation_predictor",
        "//modules/prediction/predictor/free_move:free_move_predictor",
        "//modules/prediction/predictor/interaction:interaction_predictor",
        "//modules/prediction/predictor/junction:junction_predictor",
        "//modules/prediction/predictor/lane_sequence:lane_sequence_predictor",
        "//modules/prediction/predictor/move_sequence:move_sequence_predictor",
        "//modules/prediction/predictor/single_lane:single_lane_predictor",
        "//modules/prediction/proto:prediction_conf_proto",
        "//modules/prediction/scenario:scenario_manager",
    ],
)

cc_test(
    name = "predictor_manager_test",
    size = "small",
    srcs = ["predictor_manager_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/predictor:predictor_manager",
    ],
)

cc_library(
    name = "predictor",
    srcs = ["predictor.cc"],
    hdrs = ["predictor.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/container/adc_trajectory:adc_trajectory_container",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/container/obstacles:obstacles_container",
    ],
)

cpplint()
