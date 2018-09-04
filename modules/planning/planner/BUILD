load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planner",
    hdrs = [
        "planner.h",
    ],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:frame_open_space",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario_lib",
        "//modules/planning/scenarios:scenario_manager",
    ],
)

cc_library(
    name = "planner_dispatcher",
    srcs = [
        "navi_planner_dispatcher.cc",
        "planner_dispatcher.cc",
        "std_planner_dispatcher.cc",
    ],
    hdrs = [
        "navi_planner_dispatcher.h",
        "planner_dispatcher.h",
        "std_planner_dispatcher.h",
    ],
    deps = [
        "//modules/common/status",
        "//modules/common/util",
        "//modules/planning/planner/em:em_planner",
        "//modules/planning/planner/lattice:lattice_planner",
        "//modules/planning/planner/navi:navi_planner",
        "//modules/planning/planner/open_space:open_space_planner",
        "//modules/planning/planner/rtk:rtk_planner",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_test(
    name = "std_planner_dispatcher_test",
    size = "small",
    srcs = [
        "std_planner_dispatcher_test.cc",
    ],
    deps = [
        ":planner_dispatcher",
        "@gtest//:main",
    ],
)

cc_test(
    name = "navi_planner_dispatcher_test",
    size = "small",
    srcs = [
        "navi_planner_dispatcher_test.cc",
    ],
    deps = [
        ":planner_dispatcher",
        "@gtest//:main",
    ],
)

cpplint()
