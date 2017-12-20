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
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/planner",
        "//modules/planning/proto:planning_proto",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "warm_up_problem_interface",
    srcs = [
        "warm_up_problem_interface.cc",
    ],
    hdrs = [
        "warm_up_problem_interface.h",
    ],
    deps = [
        "//modules/common/math",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planning_proto",
        "@eigen//:eigen",
        "@ipopt//:ipopt",
    ],
)

cc_library(
    name = "distance_approach_problem_interface",
    srcs = [
        "distance_approach_problem_interface.cc",
    ],
    hdrs = [
        "distance_approach_problem_interface.h",
    ],
    deps = [
        "//modules/common/math",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planning_proto",
        "@eigen//:eigen",
        "@ipopt//:ipopt",
    ],
)

cpplint()
