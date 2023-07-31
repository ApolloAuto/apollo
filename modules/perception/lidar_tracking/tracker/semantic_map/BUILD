load("@rules_cc//cc:defs.bzl", "cc_library", "cc_binary")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_evaluator_manager",
    srcs = ["evaluator_manager.cc"],
    hdrs = ["evaluator_manager.h"],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "external_perception_evaluator_manager_lib",
    ],
)

cc_binary(
    name = "libexternal_perception_evaluator_manager_lib.so",
    deps = [
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:semantic_map",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator",
        "//modules/prediction/evaluator/vehicle:semantic_lstm_evaluator",
    ],
    linkshared = True,
    linkstatic = True
)

cc_library(
    name = "external_perception_evaluator_manager_lib",
    srcs = [":libexternal_perception_evaluator_manager_lib.so"],
    deps = [
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:semantic_map",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator",
        "//modules/prediction/evaluator/vehicle:semantic_lstm_evaluator",
    ]
)

apollo_package()
cpplint()
