load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planner",
    hdrs = ["planner.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios:scenario_manager",
    ],
)

cc_library(
    name = "planner_dispatcher",
    srcs = [
        "navi_planner_dispatcher.cc",
        "on_lane_planner_dispatcher.cc",
        "planner_dispatcher.cc",
    ],
    hdrs = [
        "navi_planner_dispatcher.h",
        "on_lane_planner_dispatcher.h",
        "planner_dispatcher.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        "//modules/common/status",
        "//modules/common/util",
        "//modules/planning/planner/lattice:lattice_planner",
        "//modules/planning/planner/navi:navi_planner",
        "//modules/planning/planner/public_road:public_road_planner",
        "//modules/planning/planner/rtk:rtk_planner",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_test(
    name = "on_lane_planner_dispatcher_test",
    size = "small",
    srcs = ["on_lane_planner_dispatcher_test.cc"],
    deps = [
        ":planner_dispatcher",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "navi_planner_dispatcher_test",
    size = "small",
    srcs = ["navi_planner_dispatcher_test.cc"],
    deps = [
        ":planner_dispatcher",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
