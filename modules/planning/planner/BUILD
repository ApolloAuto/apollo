package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lib_planner",
    srcs = [
        "rtk_replay_planner.cc",
    ],
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/util",
        "//modules/common/vehicle_state",
        "//modules/planning/common:lib_planning_common",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "rtk_replay_planner_test",
    size = "small",
    srcs = [
        "rtk_replay_planner_test.cc",
    ],
    data = ["//modules/planning:planning_testdata"],
    deps = [
        ":lib_planner",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/planning/common:lib_planning_common",
        "@gtest//:main",
    ],
)
