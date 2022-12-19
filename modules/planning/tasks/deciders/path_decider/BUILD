load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "path_decider",
    srcs = ["path_decider.cc"],
    hdrs = ["path_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common_msgs/planning_msgs:decision_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/tasks:task",
    ],
)

cpplint()
