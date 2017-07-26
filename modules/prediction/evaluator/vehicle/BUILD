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
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/common:prediction_util",
        "//modules/prediction/container/obstacles:obstacle",
        "//modules/prediction/evaluator",
        "//modules/prediction/proto:fnn_vehicle_model_proto",
        "//modules/prediction/proto:lane_graph_proto",
    ],
)

cpplint()
