load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/proto:proto.bzl", "apollo_py_binary")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools/platform:build_defs.bzl", "if_gpu")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

PLANNING_FOPENMP = [
    "-DMODULE_NAME=\\\"planning\\\"",
    "-fopenmp",
]

apollo_cc_library(
    name = "apollo_planning_open_space",
    srcs = [
        "coarse_trajectory_generator/grid_search.cc",
        "coarse_trajectory_generator/hybrid_a_star.cc",
        "coarse_trajectory_generator/node3d.cc",
        "coarse_trajectory_generator/reeds_shepp_path.cc",
        "trajectory_smoother/distance_approach_ipopt_cuda_interface.cc",
        "trajectory_smoother/distance_approach_ipopt_fixed_dual_interface.cc",
        "trajectory_smoother/distance_approach_ipopt_fixed_ts_interface.cc",
        "trajectory_smoother/distance_approach_ipopt_interface.cc",
        "trajectory_smoother/distance_approach_ipopt_relax_end_interface.cc",
        "trajectory_smoother/distance_approach_ipopt_relax_end_slack_interface.cc",
        "trajectory_smoother/distance_approach_problem.cc",
        "trajectory_smoother/dual_variable_warm_start_ipopt_interface.cc",
        "trajectory_smoother/dual_variable_warm_start_ipopt_qp_interface.cc",
        "trajectory_smoother/dual_variable_warm_start_osqp_interface.cc",
        "trajectory_smoother/dual_variable_warm_start_problem.cc",
        "trajectory_smoother/dual_variable_warm_start_slack_osqp_interface.cc",
        "trajectory_smoother/iterative_anchoring_smoother.cc",
        "utils/open_space_fallback_util.cc",
        "utils/open_space_roi_util.cc",
        "utils/open_space_trajectory_optimizer_util.cc",
    ],
    hdrs = [
        "coarse_trajectory_generator/grid_search.h",
        "coarse_trajectory_generator/hybrid_a_star.h",
        "coarse_trajectory_generator/node3d.h",
        "coarse_trajectory_generator/reeds_shepp_path.h",
        "trajectory_smoother/distance_approach_interface.h",
        "trajectory_smoother/distance_approach_ipopt_cuda_interface.h",
        "trajectory_smoother/distance_approach_ipopt_fixed_dual_interface.h",
        "trajectory_smoother/distance_approach_ipopt_fixed_ts_interface.h",
        "trajectory_smoother/distance_approach_ipopt_interface.h",
        "trajectory_smoother/distance_approach_ipopt_relax_end_interface.h",
        "trajectory_smoother/distance_approach_ipopt_relax_end_slack_interface.h",
        "trajectory_smoother/distance_approach_problem.h",
        "trajectory_smoother/dual_variable_warm_start_ipopt_interface.h",
        "trajectory_smoother/dual_variable_warm_start_ipopt_qp_interface.h",
        "trajectory_smoother/dual_variable_warm_start_osqp_interface.h",
        "trajectory_smoother/dual_variable_warm_start_problem.h",
        "trajectory_smoother/dual_variable_warm_start_slack_osqp_interface.h",
        "trajectory_smoother/iterative_anchoring_smoother.h",
        "utils/open_space_fallback_util.h",
        "utils/open_space_roi_util.h",
        "utils/open_space_trajectory_optimizer_util.h",
    ],
    copts = [
        "-fopenmp",
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    linkopts = ["-lgomp"],
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util:common_util",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/planning/planning_open_space/proto:planner_open_space_config_proto",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "@com_github_gflags_gflags//:gflags",
        "@com_google_absl//:absl",
        "@com_google_googletest//:gtest",
        "@com_google_googletest//:gtest_main",
        "@com_google_protobuf//:protobuf",
        "@eigen",
        "@ipopt",
        "@osqp",
    ] + if_gpu([":planning_block"]),
)

cuda_library(
    name = "planning_block",
    srcs = ["trajectory_smoother/planning_block.cu"],
    hdrs = ["trajectory_smoother/planning_block.h"],
    mandatory = False,
    deps = [
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudart",
    ],
)

filegroup(
    name = "planning_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

apollo_cc_test(
    name = "distance_approach_ipopt_interface_test",
    size = "small",
    srcs = ["trajectory_smoother/distance_approach_ipopt_interface_test.cc"],
    copts = PLANNING_FOPENMP,
    linkopts = ["-lgomp"],
    deps = [
        ":apollo_planning_open_space",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "distance_approach_ipopt_cuda_interface_test",
    size = "small",
    srcs = ["trajectory_smoother/distance_approach_ipopt_cuda_interface_test.cc"],
    copts = PLANNING_FOPENMP,
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":apollo_planning_open_space",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "distance_approach_problem_test",
    size = "small",
    srcs = ["trajectory_smoother/distance_approach_problem_test.cc"],
    copts = PLANNING_FOPENMP,
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":apollo_planning_open_space",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "dual_variable_warm_start_ipopt_interface_test",
    size = "small",
    srcs = ["trajectory_smoother/dual_variable_warm_start_ipopt_interface_test.cc"],
    deps = [
        ":apollo_planning_open_space",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "dual_variable_warm_start_osqp_interface_test",
    size = "small",
    srcs = ["trajectory_smoother/dual_variable_warm_start_osqp_interface_test.cc"],
    linkstatic = True,
    deps = [
        ":apollo_planning_open_space",
        "@com_google_googletest//:gtest_main",
        "@ipopt",
    ],
)

apollo_cc_test(
    name = "dual_variable_warm_start_problem_test",
    size = "small",
    srcs = ["trajectory_smoother/dual_variable_warm_start_problem_test.cc"],
    deps = [
        ":apollo_planning_open_space",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "reeds_shepp_path_test",
    size = "small",
    srcs = ["coarse_trajectory_generator/reeds_shepp_path_test.cc"],
    linkopts = ["-lgomp"],
    deps = [
        ":apollo_planning_open_space",
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/planning/planning_open_space/proto:planner_open_space_config_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_binary(
    name = "hybrid_a_star_wrapper_lib.so",
    srcs = ["tools/hybrid_a_star_wrapper.cc"],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":apollo_planning_open_space",
    ],
)

apollo_cc_binary(
    name = "distance_approach_problem_wrapper_lib.so",
    srcs = ["tools/distance_approach_problem_wrapper.cc"],
    copts = ["-fopenmp"],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":apollo_planning_open_space",
    ],
)

apollo_cc_binary(
    name = "open_space_roi_wrapper_lib.so",
    srcs = ["tools/open_space_roi_wrapper.cc"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        ":apollo_planning_open_space",
        "//cyber",
        "//modules/common/math",
        "//modules/map:apollo_map",
        "//modules/planning/planning_open_space/proto:planner_open_space_config_cc_proto",
        "@eigen",
    ],
)

apollo_package(enable_source=False)

cpplint()
