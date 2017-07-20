load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mlp_evaluator",
    hdrs = [
        "mlp_evaluator.h",
    ],
    srcs = [
        "mlp_evaluator.cc",
    ],
    deps = [
        "//modules/prediction/evaluator:evaluator",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/proto:lane_graph_proto",
        "//modules/prediction/common:prediction_common",
        "//modules/common/math:math_utils",
        "//modules/prediction/proto:fnn_vehicle_model_proto",
        "//modules/prediction/common:prediction_util",
    ],
)

cpplint()
