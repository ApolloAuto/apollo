load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ])
)

apollo_plugin(
    name = "liblane_follow_scenario.so",
    srcs = [
        "lane_follow_scenario.cc",
        "lane_follow_stage.cc",
    ],
    hdrs = [
        "lane_follow_scenario.h",
        "lane_follow_stage.h",
    ],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map:apollo_map",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "lane_follow_scenario_test",
    size = "small",
    srcs = ["lane_follow_scenario_test.cc"],
    
    linkopts = ["-lgomp"],
    deps = [
        ":lane_follow_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
