load("//tools:cpplint.bzl", "cpplint")

load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "open_space_trajectory_partition",
    srcs = [
        "open_space_trajectory_partition.cc",
    ],
    hdrs = [
        "open_space_trajectory_partition.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"", 
    ],
    deps = [
        "//modules/common/status",
        "//modules/planning/tasks/optimizers:trajectory_optimizer",
        "//modules/planning/tasks:task",
        "//modules/planning/proto:planning_config_proto",
    ],
)

cc_test(
    name = "open_space_trajectory_partition_test",
    size = "small",
    srcs = [
        "open_space_trajectory_partition_test.cc",
    ],
    deps = [
        ":open_space_trajectory_partition",
        "@gtest//:main",
    ],
)

cpplint()
