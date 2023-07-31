load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

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
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common:trajectory_evaluator",
        "//modules/planning/planning_base/common/util:util_lib",
        "//modules/planning/planning_base/proto:learning_data_cc_proto",
    ],
)

cc_library(
    name = "evaluator_logger",
    hdrs = ["evaluator_logger.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/planning/planning_base/common:planning_gflags",
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
        "//modules/planning/planning_base/common:feature_output",
        "//modules/planning/planning_base/common:message_process",
        "//modules/planning/planning_base/common:planning_gflags",
    ],
)

apollo_package()
cpplint()
