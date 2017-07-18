load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_evaluator",
    hdrs = [
        "vehicle_evaluator.h",
    ],
    srcs = [
        "vehicle_evaluator.cc",
    ],
    deps = [
        "//modules/prediction/evaluator:evaluator",
        "//modules/prediction/container/obstacles:obstacle",
    ],
)

cpplint()
