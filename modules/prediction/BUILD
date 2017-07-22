load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "prediction_lib",
    srcs = ["prediction.cc"],
    hdrs = ["prediction.h"],
    deps = [
        "//modules/common",
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/util:util",
        "//modules/common/adapters:adapter_manager",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/evaluator:evaluator_manager",
        "//modules/prediction/predictor:predictor_manager",
        "//modules/perception/proto:perception_proto",
        "//modules/prediction/common:prediction_gflags",
        "//modules/prediction/proto:prediction_proto",
        "//modules/prediction/proto:prediction_conf_proto",
    ],
)

cc_binary(
    name = "prediction",
    srcs = ["main.cc"],
    deps = [
        "//modules/prediction:prediction_lib",
        "//external:gflags",
    ],
)

cpplint()
