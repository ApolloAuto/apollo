load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "open_space_trajectory_provider",
    srcs = [
        "open_space_trajectory_provider.cc",
    ],
    hdrs = [
        "open_space_trajectory_provider.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/tasks/optimizers:trajectory_optimizer",
        "//modules/common/status",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/tasks:task",
    ],
)

cc_test(
    name = "open_space_trajectory_provider_test",
    size = "small",
    srcs = [
        "open_space_trajectory_provider_test.cc",
    ],
    deps = [
        ":open_space_trajectory_provider",
        "@gtest//:main",
    ],
)

cpplint()
