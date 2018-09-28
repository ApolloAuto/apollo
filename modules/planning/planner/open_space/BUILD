load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "open_space_planner",
    srcs = [
        "open_space_planner.cc",
    ],
    hdrs = [
        "open_space_planner.h",
    ],
    deps = [
        "warm_start_problem",
        "distance_approach_problem",
        "hybrid_a_star",
        "//external:gflags",
        "//framework:cybertron",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_common",
        "//modules/planning/planner",
        "//modules/planning/common:frame_open_space",
        "//modules/planning/proto:planning_proto",
        "@eigen//:eigen",
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
        "//modules/common/util",
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
        "//framework:cybertron",
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/proto:planning_config_proto",
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
    ],
)

cc_test(
    name = "open_space_planner_test",
    size = "small",
    srcs = [
        "open_space_planner_test.cc",
    ],
    deps = [
        "@gtest//:main",
        ":open_space_planner",
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
        "//framework:cybertron",
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
    "//modules/planning/planner/open_space:hybrid_a_star",
    "@gtest//:main",
    ],
)

cc_binary(
    name = "hybrid_a_star_wrapper_lib.so",
    linkshared = True,
    srcs = [
        "hybrid_a_star_wrapper.cc",
    ],
    deps = [
        "//modules/planning/planner/open_space:hybrid_a_star",
    ],
)

cpplint()
