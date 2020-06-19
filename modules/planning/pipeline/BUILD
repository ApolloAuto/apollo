load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "evaluator",
    srcs = ["evaluator.cc"],
    hdrs = ["evaluator.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":evaluator_logger",
        "//cyber",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/common:trajectory_evaluator",
        "//modules/planning/proto:learning_data_proto",
    ],
)

cc_library(
    name = "evaluator_logger",
    hdrs = ["evaluator_logger.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_binary(
    name = "evaluate_trajectory",
    srcs = ["evaluate_trajectory.cc"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        ":evaluator",
    ],
)

cc_binary(
    name = "record_to_learning_data",
    srcs = ["record_to_learning_data.cc"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:feature_output",
        "//modules/planning/common:message_process",
    ],
)

cpplint()
