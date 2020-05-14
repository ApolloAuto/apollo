load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_evaluator_manager",
    srcs = ["evaluator_manager.cc"],
    hdrs = ["evaluator_manager.h"],
    
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:semantic_map",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator/vehicle:semantic_lstm_evaluator",
        "//modules/prediction/evaluator:evaluator",
    ],
)

cpplint()
