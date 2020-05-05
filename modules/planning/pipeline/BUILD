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
        "//modules/localization/proto:localization_proto",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/util:math_util_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/proto:learning_data_proto",
        "//modules/prediction/proto:prediction_proto",
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
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:message_process",
        "//third_party:boost",
        "@com_google_absl//absl/strings",
    ],
)

cpplint()
