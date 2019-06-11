load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obstacles_prioritizer",
    srcs = ["obstacles_prioritizer.cc"],
    hdrs = ["obstacles_prioritizer.h"],
    copts = ['-DMODULE_NAME=\\"prediction\\"'],
    deps = [
        "//modules/prediction/common:environment_features",
        "//modules/prediction/container:container_manager",
        "//modules/prediction/container/obstacles:obstacle_clusters",
        "//modules/prediction/container/obstacles:obstacles_container",
        "//modules/prediction/container/pose:pose_container",
        "//modules/prediction/scenario/scenario_features",
        "//modules/prediction/scenario/scenario_features:cruise_scenario_features",
    ],
)

cpplint()
