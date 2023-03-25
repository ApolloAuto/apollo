load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//third_party/gpus:common.bzl", "gpu_library", "if_gpu", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]
PLANNING_FOPENMP = ["-DMODULE_NAME=\\\"planning\\\"","-fopenmp"]

cc_library(
    name = "dual_variable_warm_start_problem",
    srcs = ["dual_variable_warm_start_problem.cc"],
    hdrs = ["dual_variable_warm_start_problem.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":dual_variable_warm_start_ipopt_interface",
        ":dual_variable_warm_start_ipopt_qp_interface",
        ":dual_variable_warm_start_osqp_interface",
        ":dual_variable_warm_start_slack_osqp_interface",
        "//cyber",
        "//modules/common/util:util_tool",
    ],
)

cc_library(
    name = "dual_variable_warm_start_ipopt_interface",
    srcs = ["dual_variable_warm_start_ipopt_interface.cc"],
    hdrs = ["dual_variable_warm_start_ipopt_interface.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "dual_variable_warm_start_ipopt_qp_interface",
    srcs = ["dual_variable_warm_start_ipopt_qp_interface.cc"],
    hdrs = ["dual_variable_warm_start_ipopt_qp_interface.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "dual_variable_warm_start_osqp_interface",
    srcs = ["dual_variable_warm_start_osqp_interface.cc"],
    hdrs = ["dual_variable_warm_start_osqp_interface.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@eigen",
        "@osqp",
    ],
)

cc_library(
    name = "dual_variable_warm_start_slack_osqp_interface",
    srcs = ["dual_variable_warm_start_slack_osqp_interface.cc"],
    hdrs = ["dual_variable_warm_start_slack_osqp_interface.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@eigen",
        "@osqp",
    ],
)

cc_library(
    name = "distance_approach_problem",
    srcs = ["distance_approach_problem.cc"],
    hdrs = ["distance_approach_problem.h"],
    copts = PLANNING_FOPENMP,
    deps = [
        ":distance_approach_ipopt_cuda_interface",
        ":distance_approach_ipopt_fixed_dual_interface",
        ":distance_approach_ipopt_fixed_ts_interface",
        ":distance_approach_ipopt_interface",
        ":distance_approach_ipopt_relax_end_interface",
        ":distance_approach_ipopt_relax_end_slack_interface",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
    ],
)

cc_library(
    name = "distance_approach_ipopt_fixed_ts_interface",
    srcs = ["distance_approach_ipopt_fixed_ts_interface.cc"],
    hdrs = [
        "distance_approach_interface.h",
        "distance_approach_ipopt_fixed_ts_interface.h",
    ],
    copts = PLANNING_FOPENMP,
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "distance_approach_ipopt_fixed_dual_interface",
    srcs = ["distance_approach_ipopt_fixed_dual_interface.cc"],
    hdrs = [
        "distance_approach_interface.h",
        "distance_approach_ipopt_fixed_dual_interface.h",
    ],
    copts = PLANNING_FOPENMP,
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "distance_approach_ipopt_relax_end_interface",
    srcs = ["distance_approach_ipopt_relax_end_interface.cc"],
    hdrs = [
        "distance_approach_interface.h",
        "distance_approach_ipopt_relax_end_interface.h",
    ],
    copts = PLANNING_FOPENMP,
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "distance_approach_ipopt_relax_end_slack_interface",
    srcs = ["distance_approach_ipopt_relax_end_slack_interface.cc"],
    hdrs = [
        "distance_approach_interface.h",
        "distance_approach_ipopt_relax_end_slack_interface.h",
    ],
    copts = PLANNING_FOPENMP,
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "distance_approach_ipopt_interface",
    srcs = ["distance_approach_ipopt_interface.cc"],
    hdrs = [
        "distance_approach_interface.h",
        "distance_approach_ipopt_interface.h",
    ],
    copts = PLANNING_FOPENMP,
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "distance_approach_ipopt_cuda_interface",
    srcs = ["distance_approach_ipopt_cuda_interface.cc"],
    hdrs = [
        "distance_approach_interface.h",
        "distance_approach_ipopt_cuda_interface.h",
    ],
    copts = PLANNING_FOPENMP,
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ] + if_gpu([":planning_block"]),
)

cc_library(
    name = "iterative_anchoring_smoother",
    srcs = ["iterative_anchoring_smoother.cc"],
    hdrs = ["iterative_anchoring_smoother.h"],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/math:discrete_points_math",
        "//modules/planning/math/discretized_points_smoothing:fem_pos_deviation_smoother",
        "//modules/planning/proto:planner_open_space_config_cc_proto",
        "@eigen",
    ],
)

gpu_library(
    name = "planning_block",
    srcs = ["planning_block.cu"],
    hdrs = ["planning_block.h"],
    deps = [
    ] + if_cuda([
        "@local_config_cuda//cuda:cuda_headers",
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:rocm_headers",
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_test(
    name = "distance_approach_ipopt_interface_test",
    size = "small",
    srcs = ["distance_approach_ipopt_interface_test.cc"],
    copts = PLANNING_FOPENMP,
    linkopts = ["-lgomp"],
    deps = [
        ":distance_approach_ipopt_interface",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "distance_approach_ipopt_cuda_interface_test",
    size = "small",
    srcs = ["distance_approach_ipopt_cuda_interface_test.cc"],
    copts = PLANNING_FOPENMP,
    linkopts = ["-lgomp"],
    deps = [
        ":distance_approach_ipopt_cuda_interface",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
    linkstatic = True,
)

cc_test(
    name = "distance_approach_problem_test",
    size = "small",
    srcs = ["distance_approach_problem_test.cc"],
    copts = PLANNING_FOPENMP,
    linkopts = ["-lgomp"],
    deps = [
        ":distance_approach_problem",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
    linkstatic = True,
)

cc_test(
    name = "dual_variable_warm_start_ipopt_interface_test",
    size = "small",
    srcs = ["dual_variable_warm_start_ipopt_interface_test.cc"],
    deps = [
        ":dual_variable_warm_start_ipopt_interface",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "dual_variable_warm_start_osqp_interface_test",
    size = "small",
    srcs = ["dual_variable_warm_start_osqp_interface_test.cc"],
    deps = [
        ":dual_variable_warm_start_ipopt_qp_interface",
        ":dual_variable_warm_start_osqp_interface",
        "@com_google_googletest//:gtest_main",
        "@ipopt",
    ],
    linkstatic = True,
)

cc_test(
    name = "dual_variable_warm_start_problem_test",
    size = "small",
    srcs = ["dual_variable_warm_start_problem_test.cc"],
    deps = [
        ":dual_variable_warm_start_problem",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
