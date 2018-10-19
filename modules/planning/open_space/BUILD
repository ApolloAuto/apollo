load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "node3d",
    srcs = ["node3d.cc"],
    hdrs = ["node3d.h"],
    deps = ["//cyber",
            "//modules/common/math",
            "//modules/planning/common:obstacle",
            "//modules/planning/constraint_checker:collision_checker",
            "//modules/planning/proto:planner_open_space_config_proto",],
)

cc_library(
    name = "reeds_shepp_path",
    srcs = ["reeds_shepp_path.cc"],
    hdrs = ["reeds_shepp_path.h"],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/planning/open_space:node3d"],
)

cc_library(
    name = "open_space_utils",
    deps = ["//modules/planning/open_space:reeds_shepp_path",
            "//modules/planning/open_space:node3d",]
)

cc_test(
    name = "reeds_shepp_path_test",
    size = "small",
    srcs = ["reeds_shepp_path_test.cc"],
    deps = [
        "//cyber",
        "//modules/planning/open_space:open_space_utils",
        "//modules/common/math",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/proto:planner_open_space_config_proto",
        "@gtest//:main",],
)

cc_test(
    name = "node3d_test",
    size = "small",
    srcs = ["node3d_test.cc"],
    deps = [
        "//cyber",
        "//modules/planning/open_space:open_space_utils",
        "@gtest//:main",
    ],
)

cc_library(
    name = "warm_start_problem",
    srcs = [
        "warm_start_problem.cc",
    ],
    hdrs = [
        "warm_start_problem.h",
    ],
    deps = [
        "warm_start_ipopt_interface",
    ],
)

cc_library(
    name = "warm_start_ipopt_interface",
    srcs = [
        "warm_start_ipopt_interface.cc",
    ],
    hdrs = [
        "warm_start_ipopt_interface.h",
    ],
    deps = [
        "//modules/common/math",
        "//modules/common/util",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/proto:planner_open_space_config_proto",
        "@eigen//:eigen",
        "@ipopt//:ipopt",
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
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planner_open_space_config_proto",
        "distance_approach_ipopt_interface",
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
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/common/configs:vehicle_config_helper",
        "@eigen//:eigen",
        "@ipopt//:ipopt",
    ],
)

cc_test(
    name = "distance_approach_ipopt_interface_test",
    size = "small",
    srcs = [
        "distance_approach_ipopt_interface_test.cc",
    ],
    deps = [
        "@gtest//:main",
        ":distance_approach_ipopt_interface",
        "//cyber",
    ],
)

cc_test(
    name = "distance_approach_problem_test",
    size = "small",
    srcs = [
        "distance_approach_problem_test.cc",
    ],
    deps = [
        "@gtest//:main",
        ":distance_approach_problem",
        "//cyber",
    ],
)

cc_test(
    name = "warm_start_ipopt_interface_test",
    size = "small",
    srcs = [
        "warm_start_ipopt_interface_test.cc",
    ],
    deps = [
        "@gtest//:main",
        ":warm_start_ipopt_interface",
    ],
)

cc_test(
    name = "warm_start_problem_test",
    size = "small",
    srcs = [
        "warm_start_problem_test.cc",
    ],
    deps = [
        "@gtest//:main",
        ":warm_start_problem",
    ],
)

cc_library(
    name = "hybrid_a_star",
    srcs = [
        "hybrid_a_star.cc",
    ],
    hdrs = [
        "hybrid_a_star.h",
    ],
    deps = [
        "//cyber",
        "//modules/planning/common:obstacle",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/proto:planner_open_space_config_proto",
        "//modules/planning/open_space:open_space_utils",
    ],
)

cc_test(
    name = "hybrid_a_star_test",
    size = "medium",
    srcs = [
        "hybrid_a_star_test.cc",
    ],
    deps = [
    "//modules/planning/open_space:hybrid_a_star",
    "@gtest//:main",
    ],
)

cc_binary(
    name = "hybrid_a_star_wrapper_lib.so",
    linkshared = True,
    linkstatic = False,
    srcs = [
        "hybrid_a_star_wrapper.cc",
    ],
    deps = [
        "//modules/planning/open_space:hybrid_a_star",
    ],
)

cc_binary(
    name = "distance_approach_problem_wrapper_lib.so",
    linkshared = True,
    linkstatic = False,
    srcs = [
        "distance_approach_problem_wrapper.cc",
    ],
    deps = [
        "//modules/planning/open_space:hybrid_a_star",
        "//modules/planning/open_space:distance_approach_problem",
    ],
)

cpplint()