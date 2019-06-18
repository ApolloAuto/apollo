load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mlp_evaluator",
    srcs = [
        "mlp_evaluator.cc",
    ],
    hdrs = [
        "mlp_evaluator.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/common:validation_checker",
        "//modules/prediction/evaluator",
        "//modules/prediction/proto:fnn_vehicle_model_proto",
    ],
)

cc_test(
    name = "mlp_evaluator_test",
    size = "small",
    srcs = [
        "mlp_evaluator_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator/vehicle:mlp_evaluator",
        "@gtest//:main",
    ],
)

cc_library(
    name = "rnn_evaluator",
    srcs = [
        "rnn_evaluator.cc",
    ],
    hdrs = [
        "rnn_evaluator.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/evaluator",
    ],
)

cc_test(
    name = "rnn_evaluator_test",
    size = "small",
    srcs = [
        "rnn_evaluator_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator/vehicle:rnn_evaluator",
        "@gtest//:main",
    ],
)

cc_library(
    name = "cost_evaluator",
    srcs = [
        "cost_evaluator.cc",
    ],
    hdrs = [
        "cost_evaluator.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/evaluator",
    ],
)

cc_test(
    name = "cost_evaluator_test",
    size = "small",
    srcs = [
        "cost_evaluator_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator/vehicle:cost_evaluator",
        "@gtest//:main",
    ],
)

cc_library(
    name = "junction_mlp_evaluator",
    srcs = [
        "junction_mlp_evaluator.cc",
    ],
    hdrs = [
        "junction_mlp_evaluator.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/common/math:geometry",
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/evaluator",
    ] + select({
        "//tools/platforms:use_gpu": [
            "@pytorch",
        ],
        "//conditions:default": [
            "@pytorch",
        ],
    }),
)

cc_test(
    name = "junction_mlp_evaluator_test",
    size = "small",
    srcs = [
        "junction_mlp_evaluator_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator/vehicle:junction_mlp_evaluator",
        "@gtest//:main",
    ],
)

cc_library(
    name = "junction_map_evaluator",
    srcs = [
        "junction_map_evaluator.cc",
    ],
    hdrs = [
        "junction_map_evaluator.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/common:semantic_map",
        "//modules/prediction/evaluator",
    ] + select({
        "//tools/platforms:use_gpu": [
            "@pytorch",
        ],
        "//conditions:default": [
            "@pytorch",
        ],
    }),
)

cc_library(
    name = "cruise_mlp_evaluator",
    srcs = [
        "cruise_mlp_evaluator.cc",
    ],
    hdrs = [
        "cruise_mlp_evaluator.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/evaluator",
    ] + select({
        "//tools/platforms:use_gpu": [
            "@pytorch",
        ],
        "//conditions:default": [
            "@pytorch",
        ],
    }),
)

cc_test(
    name = "cruise_mlp_evaluator_test",
    size = "small",
    srcs = [
        "cruise_mlp_evaluator_test.cc",
    ],
    data = [
        "//modules/prediction:prediction_data",
        "//modules/prediction:prediction_testdata",
    ],
    deps = [
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/evaluator/vehicle:cruise_mlp_evaluator",
    ],
)

cc_library(
    name = "lane_scanning_evaluator",
    srcs = [
        "lane_scanning_evaluator.cc",
    ],
    hdrs = [
        "lane_scanning_evaluator.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/container:container_manager",
        "//modules/prediction/evaluator",
    ] + select({
        "//tools/platforms:use_gpu": [
            "@pytorch",
        ],
        "//conditions:default": [
            "@pytorch",
        ],
    }),
)

cc_library(
    name = "lane_aggregating_evaluator",
    srcs = [
        "lane_aggregating_evaluator.cc",
    ],
    hdrs = [
        "lane_aggregating_evaluator.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    deps = [
        "//modules/prediction/container:container_manager",
        "//modules/prediction/evaluator",
    ] + select({
        "//tools/platforms:use_gpu": [
            "@pytorch",
        ],
        "//conditions:default": [
            "@pytorch",
        ],
    }),
)

cpplint()
