load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_plugin", "apollo_cc_library", "apollo_cc_test", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ])
)

apollo_plugin(
    name = "libnavi_planner.so",
    srcs = [
        "navi_planner.cc",
    ],
    hdrs = [
        "navi_planner.h",
    ],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        ":navi_planner_base",
    ],
)

apollo_cc_library(
    name = "navi_planner_base",
    srcs = [
        "decider/navi_obstacle_decider.cc",
        "decider/navi_path_decider.cc",
        "decider/navi_speed_decider.cc",
        "decider/navi_speed_ts_graph.cc",
        "decider/navi_task.cc",
    ],
    hdrs = [
        "decider/navi_obstacle_decider.h",
        "decider/navi_path_decider.h",
        "decider/navi_speed_decider.h",
        "decider/navi_speed_ts_graph.h",
        "decider/navi_task.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/planners/navi/proto:planner_config_cc_proto",
    ],
)


apollo_cc_test(
    name = "navi_planner_test",
    size = "small",
    srcs = ["navi_planner_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":navi_planner_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "navi_decider_test",
    size = "small",
    srcs = [
        "decider/navi_obstacle_decider_test.cc",
        "decider/navi_path_decider_test.cc",
        "decider/navi_speed_decider_test.cc",
        "decider/navi_speed_ts_graph_test.cc",
    ],
    deps = [
        ":navi_planner_base",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/common/util:common_util",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
