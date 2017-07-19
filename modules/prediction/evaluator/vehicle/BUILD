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
    ],
)

cpplint()
