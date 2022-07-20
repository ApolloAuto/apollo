load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]
FOPENMP_COPTS = ["-fopenmp"]

cc_library(
    name = "open_space_trajectory_provider",
    srcs = ["open_space_trajectory_provider.cc"],
    hdrs = ["open_space_trajectory_provider.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        ":open_space_trajectory_optimizer",
        "//modules/common/status",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:trajectory_stitcher",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/optimizers:trajectory_optimizer",
    ] + if_gpu(["@local_config_cuda//cuda:cudart"]),
)

cc_library(
    name = "open_space_trajectory_optimizer",
    srcs = ["open_space_trajectory_optimizer.cc"],
    hdrs = ["open_space_trajectory_optimizer.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:frame",
        "//modules/planning/open_space/coarse_trajectory_generator:hybrid_a_star",
        "//modules/planning/open_space/trajectory_smoother:distance_approach_problem",
        "//modules/planning/open_space/trajectory_smoother:dual_variable_warm_start_problem",
        "//modules/planning/open_space/trajectory_smoother:iterative_anchoring_smoother",
        "//modules/planning/proto:planning_config_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cc_test(
    name = "open_space_trajectory_provider_test",
    size = "small",
    srcs = ["open_space_trajectory_provider_test.cc"],
    copts = FOPENMP_COPTS,
    linkopts = ["-lgomp"],
    deps = [
        ":open_space_trajectory_provider",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "open_space_trajectory_optimizer_test",
    size = "small",
    srcs = ["open_space_trajectory_optimizer_test.cc"],
    copts = FOPENMP_COPTS,
    linkopts = ["-lgomp"],
    deps = [
        ":open_space_trajectory_optimizer",
        "@com_google_googletest//:gtest_main",
    ] + if_gpu(["@local_config_cuda//cuda:cudart"]),
)

cpplint()
