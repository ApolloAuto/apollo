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
        "//modules/common/math:math_utils",
        "//modules/common/util:util",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/evaluator",
        "//modules/prediction/proto:fnn_vehicle_model_proto",
        "//modules/prediction/proto:lane_graph_proto",
        "//modules/map/proto:map_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "mlp_evaluator_test",
    size = "small",
    srcs = [
        "mlp_evaluator_test.cc",
    ],
    data = ["//modules/prediction:prediction_data"],
    deps = [
        "//modules/prediction/evaluator/vehicle:mlp_evaluator",
        "@gtest//:main",
        "//modules/prediction/common:prediction_gflags",
    ],
)

cpplint()
