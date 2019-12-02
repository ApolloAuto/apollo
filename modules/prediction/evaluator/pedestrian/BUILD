load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pedestrian_interaction_evaluator",
    srcs = ["pedestrian_interaction_evaluator.cc"],
    hdrs = ["pedestrian_interaction_evaluator.h"],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/container:container_manager",
        "//modules/prediction/evaluator",
        "//modules/prediction/container/obstacles:obstacles_container",
    ] + select({
        "//tools/platforms:use_gpu": [
            "@pytorch_gpu",
        ],
        "//conditions:default": [
            "@pytorch",
        ],
    }),
)

cc_test(
    name = "pedestrian_interaction_evaluator_test",
    size = "small",
    srcs = ["pedestrian_interaction_evaluator_test.cc"],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        ":pedestrian_interaction_evaluator",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/container/obstacles:obstacles_container",
        "@gtest//:main",
    ],
)

cpplint()
