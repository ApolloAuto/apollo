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
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_common",
        "//modules/planning/planner",
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
        "//modules/common/math",
        "//modules/common/util",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:planning_proto",
        "//modules/common/configs:vehicle_config_helper",
        "@eigen//:eigen",
        "@ipopt//:ipopt",
    ],
)

cpplint()
