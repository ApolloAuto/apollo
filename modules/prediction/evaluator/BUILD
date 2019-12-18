load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "evaluator_manager",
    srcs = ["evaluator_manager.cc"],
    hdrs = ["evaluator_manager.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_thread_pool",
        "//modules/prediction/common:semantic_map",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator/cyclist:cyclist_keep_lane_evaluator",
        "//modules/prediction/evaluator/pedestrian:pedestrian_interaction_evaluator",
        "//modules/prediction/evaluator/vehicle:cost_evaluator",
        "//modules/prediction/evaluator/vehicle:cruise_mlp_evaluator",
        "//modules/prediction/evaluator/vehicle:junction_map_evaluator",
        "//modules/prediction/evaluator/vehicle:junction_mlp_evaluator",
        "//modules/prediction/evaluator/vehicle:lane_aggregating_evaluator",
        "//modules/prediction/evaluator/vehicle:lane_scanning_evaluator",
        "//modules/prediction/evaluator/vehicle:mlp_evaluator",
        "//modules/prediction/evaluator/vehicle:semantic_lstm_evaluator",
        "//modules/prediction/proto:prediction_conf_proto",
    ],
)

cc_test(
    name = "evaluator_manager_test",
    size = "small",
    srcs = ["evaluator_manager_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/evaluator:evaluator_manager",
    ],
)

cc_library(
    name = "evaluator",
    hdrs = ["evaluator.h"],
    deps = [
        "//modules/prediction/container/obstacles:obstacle",
    ],
)

cpplint()
