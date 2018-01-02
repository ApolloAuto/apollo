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
        "//modules/common/configs:config_gflags",
        "//modules/common/math:math_utils",
        "//modules/common/util",
        "//modules/map/proto:map_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_util",
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
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/evaluator",
        "//modules/prediction/proto:network_model_proto",
        "//modules/prediction/common:prediction_map",
        "//modules/prediction/network/rnn_model:rnn_model",
        "@eigen//:eigen",
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


cpplint()
