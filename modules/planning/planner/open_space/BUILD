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
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//external:gflags",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_common",
        "//modules/planning/open_space/constraints_formulation:open_space_ROI",
        "//modules/planning/open_space:open_space_trajectory_generator",
        "//modules/planning/open_space/trajectory_partition:trajectory_partitioner",
        "//modules/planning/planner",
        "//modules/planning/proto:planning_proto",
        "@eigen",
    ],
)

cc_test(
    name = "open_space_planner_test",
    size = "small",
    srcs = [
        "open_space_planner_test.cc",
    ],
    deps = [
        ":open_space_planner",
        "@gtest//:main",
    ],
)

cpplint()
