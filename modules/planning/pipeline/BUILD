load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
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
        "//cyber",
        "//modules/localization/proto:localization_cc_proto",
        "//modules/perception/proto:perception_obstacle_cc_proto",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/util:math_util_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/proto:learning_data_cc_proto",
        "//modules/prediction/proto:prediction_obstacle_cc_proto",
        "//modules/prediction/util:data_extraction",
        "//third_party:boost",
        "@com_google_absl//absl/strings",
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
        "//third_party:boost",
        "@com_google_absl//absl/strings",
    ],
)

cc_binary(
    name = "record_to_learning_data",
    srcs = ["record_to_learning_data.cc"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
<<<<<<< HEAD
        "//modules/planning/common:feature_output",
        "//modules/planning/common:message_process",
        "//modules/planning/common:planning_gflags",
=======
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:feature_output",
        "//modules/planning/common:message_process",
>>>>>>> master
        "//third_party:boost",
        "@com_google_absl//absl/strings",
    ],
)

cpplint()
