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
    deps = [
        "//modules/common/math:geometry",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/common:validation_checker",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/evaluator",
        "//modules/prediction/proto:fnn_vehicle_model_proto",
        "//modules/prediction/proto:lane_graph_proto",
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
        "//modules/common/util",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
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
    deps = [
        "//modules/common/util",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/evaluator",
        "//modules/prediction/network/rnn_model",
        "//modules/prediction/proto:network_model_proto",
        "@eigen",
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
        "//modules/common/util",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:kml_map_based_test",
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
    deps = [
        "//modules/common/math:geometry",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        "//modules/prediction/common:feature_output",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/evaluator",
        "//modules/prediction/proto:feature_proto",
        "//modules/prediction/proto:lane_graph_proto",
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
        "//modules/common/util",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:kml_map_based_test",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/evaluator/vehicle:cost_evaluator",
        "@gtest//:main",
    ],
)

cpplint()
