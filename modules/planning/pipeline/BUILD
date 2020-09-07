load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "evaluator",
    srcs = ["evaluator.cc"],
    hdrs = ["evaluator.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":evaluator_logger",
        "//cyber",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:trajectory_evaluator",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/proto:learning_data_cc_proto",
    ],
)

cc_library(
    name = "evaluator_logger",
    hdrs = ["evaluator_logger.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_binary(
    name = "evaluate_trajectory",
    srcs = ["evaluate_trajectory.cc"],
    copts = PLANNING_COPTS,
    deps = [
        ":evaluator",
    ],
)

cc_binary(
    name = "record_to_learning_data",
    srcs = ["record_to_learning_data.cc"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:feature_output",
        "//modules/planning/common:message_process",
        "//modules/planning/common:planning_gflags",
    ],
)

cpplint()
