load("//tools:cpplint.bzl", "cpplint")

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
        "dual_variable_warm_start_ipopt_interface",
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
    name = "distance_approach_problem",
    srcs = [
        "distance_approach_problem.cc",
    ],
    hdrs = [
        "distance_approach_problem.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "distance_approach_ipopt_interface",
        "//cyber/common:log",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
    ],
)

cc_library(
    name = "distance_approach_ipopt_interface",
    srcs = [
        "distance_approach_ipopt_interface.cc",
    ],
    hdrs = [
        "distance_approach_ipopt_interface.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\"", "-fopenmp"],
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

cc_test(
    name = "distance_approach_ipopt_interface_test",
    size = "small",
    srcs = [
        "distance_approach_ipopt_interface_test.cc",
    ],
    deps = [
        ":distance_approach_ipopt_interface",
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
