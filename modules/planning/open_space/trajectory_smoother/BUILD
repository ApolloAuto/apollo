load("//tools:cpplint.bzl", "cpplint")
load("//tools:cuda_library.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dual_variable_warm_start_problem",
    srcs = [
        "dual_variable_warm_start_problem.cc",
    ],
    hdrs = [
        "dual_variable_warm_start_problem.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":dual_variable_warm_start_ipopt_interface",
        ":dual_variable_warm_start_ipopt_qp_interface",
        ":dual_variable_warm_start_osqp_interface",
    ],
)

cc_library(
    name = "dual_variable_warm_start_ipopt_interface",
    srcs = [
        "dual_variable_warm_start_ipopt_interface.cc",
    ],
    hdrs = [
        "dual_variable_warm_start_ipopt_interface.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/planning/proto:planning_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "dual_variable_warm_start_ipopt_qp_interface",
    srcs = [
        "dual_variable_warm_start_ipopt_qp_interface.cc",
    ],
    hdrs = [
        "dual_variable_warm_start_ipopt_qp_interface.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/planning/proto:planning_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "dual_variable_warm_start_osqp_interface",
    srcs = [
        "dual_variable_warm_start_osqp_interface.cc",
    ],
    hdrs = [
        "dual_variable_warm_start_osqp_interface.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/planning/proto:planning_proto",
        "@eigen",
        "@osqp",
    ],
)

cc_library(
    name = "distance_approach_problem",
    srcs = [
        "distance_approach_problem.cc",
    ],
    hdrs = [
        "distance_approach_problem.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        ":distance_approach_ipopt_cuda_interface",
        ":distance_approach_ipopt_fixed_dual_interface",
        ":distance_approach_ipopt_fixed_ts_interface",
        ":distance_approach_ipopt_interface",
        ":distance_approach_ipopt_relax_end_interface",
        "//cyber/common:log",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
    ],
)

cc_library(
    name = "distance_approach_ipopt_fixed_ts_interface",
    srcs = [
        "distance_approach_ipopt_fixed_ts_interface.cc",
    ],
    hdrs = [
        "distance_approach_interface.h",
        "distance_approach_ipopt_fixed_ts_interface.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/planning/proto:planning_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "distance_approach_ipopt_fixed_dual_interface",
    srcs = [
        "distance_approach_ipopt_fixed_dual_interface.cc",
    ],
    hdrs = [
        "distance_approach_interface.h",
        "distance_approach_ipopt_fixed_dual_interface.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/planning/proto:planning_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "distance_approach_ipopt_relax_end_interface",
    srcs = [
        "distance_approach_ipopt_relax_end_interface.cc",
    ],
    hdrs = [
        "distance_approach_interface.h",
        "distance_approach_ipopt_relax_end_interface.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/planning/proto:planning_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "distance_approach_ipopt_interface",
    srcs = [
        "distance_approach_ipopt_interface.cc",
    ],
    hdrs = [
        "distance_approach_interface.h",
        "distance_approach_ipopt_interface.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/planning/proto:planning_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "distance_approach_ipopt_cuda_interface",
    srcs = [
        "distance_approach_ipopt_cuda_interface.cc",
    ] + select({
        "//tools/platforms:use_gpu": [
            "planning_block.h",
            ":planning_block",
        ],
        "//conditions:default": [],
    }),
    hdrs = [
        "distance_approach_interface.h",
        "distance_approach_ipopt_cuda_interface.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/planning/proto:planning_proto",
        "@adolc",
        "@eigen",
        "@ipopt",
    ],
)

cc_library(
    name = "iterative_anchoring_smoother",
    srcs = [
        "iterative_anchoring_smoother.cc",
    ],
    hdrs = [
        "iterative_anchoring_smoother.h",
    ],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/math:discrete_points_math",
        "//modules/planning/math/discretized_points_smoothing:fem_pos_deviation_smoother",
        "@eigen",
    ],
)

cuda_library(
    name = "planning_block",
    srcs = [
        "planning_block.cu",
    ],
    hdrs = [
        "planning_block.h",
    ],
    deps = [
        "@cuda",
    ],
)

cc_test(
    name = "distance_approach_ipopt_interface_test",
    size = "small",
    srcs = [
        "distance_approach_ipopt_interface_test.cc",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        ":distance_approach_ipopt_interface",
        "//cyber/common:log",
        "@gtest//:main",
    ],
)

cc_test(
    name = "distance_approach_ipopt_cuda_interface_test",
    size = "small",
    srcs = [
        "distance_approach_ipopt_cuda_interface_test.cc",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        ":distance_approach_ipopt_cuda_interface",
        "//cyber/common:log",
        "@gtest//:main",
    ],
)

cc_test(
    name = "distance_approach_problem_test",
    size = "small",
    srcs = [
        "distance_approach_problem_test.cc",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        ":distance_approach_problem",
        "//cyber/common:log",
        "@gtest//:main",
    ],
)

cc_test(
    name = "dual_variable_warm_start_ipopt_interface_test",
    size = "small",
    srcs = [
        "dual_variable_warm_start_ipopt_interface_test.cc",
    ],
    deps = [
        ":dual_variable_warm_start_ipopt_interface",
        "@gtest//:main",
    ],
)

cc_test(
    name = "dual_variable_warm_start_osqp_interface_test",
    size = "small",
    srcs = [
        "dual_variable_warm_start_osqp_interface_test.cc",
    ],
    deps = [
        ":dual_variable_warm_start_ipopt_qp_interface",
        ":dual_variable_warm_start_osqp_interface",
        "@gtest//:main",
        "@ipopt",
    ],
)

cc_test(
    name = "dual_variable_warm_start_problem_test",
    size = "small",
    srcs = [
        "dual_variable_warm_start_problem_test.cc",
    ],
    deps = [
        ":dual_variable_warm_start_problem",
        "@gtest//:main",
    ],
)

cpplint()
