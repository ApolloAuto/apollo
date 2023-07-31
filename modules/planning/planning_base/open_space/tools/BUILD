load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "hybrid_a_star_wrapper_lib.so",
    srcs = ["hybrid_a_star_wrapper.cc"],
    linkshared = True,
    linkstatic = False,
    deps = [
        "//modules/planning/planning_base/open_space/coarse_trajectory_generator:hybrid_a_star",
    ],
)

cc_binary(
    name = "distance_approach_problem_wrapper_lib.so",
    srcs = ["distance_approach_problem_wrapper.cc"],
    copts = ["-fopenmp"],
    linkshared = True,
    linkstatic = False,
    deps = [
        "//modules/planning/planning_base/open_space/coarse_trajectory_generator:hybrid_a_star",
        "//modules/planning/planning_base/open_space/trajectory_smoother:distance_approach_problem",
        "//modules/planning/planning_base/open_space/trajectory_smoother:dual_variable_warm_start_problem",
    ],
)

cc_binary(
    name = "open_space_roi_wrapper_lib.so",
    srcs = ["open_space_roi_wrapper.cc"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    linkshared = True,
    linkstatic = False,
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/map/pnc_map",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/proto:planner_open_space_config_cc_proto",
        "@eigen",
    ],
)

apollo_package()
cpplint()
