load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "open_space_trajectory_generator",
    srcs = [
        "open_space_trajectory_generator.cc",
    ],
    hdrs = [
        "open_space_trajectory_generator.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/open_space/trajectory_smoother:distance_approach_problem",
        "//modules/planning/open_space/trajectory_smoother:dual_variable_warm_start_problem",
        "//modules/planning/open_space/coarse_trajectory_generator:hybrid_a_star",
        "//cyber/common:log",
        "//external:gflags",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_common",
        "//modules/planning/proto:planning_proto",
        "@eigen",
    ],
)

cpplint()
