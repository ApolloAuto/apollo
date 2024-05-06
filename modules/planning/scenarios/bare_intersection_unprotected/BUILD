load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libbare_intersection_unprotected_scenario.so",
    srcs = [
        "bare_intersection_unprotected_scenario.cc",
        "stage_approach.cc",
        "stage_intersection_cruise.cc",
    ],
    hdrs = [
        "bare_intersection_unprotected_scenario.h",
        "stage_approach.h",
        "stage_intersection_cruise.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/scenarios/bare_intersection_unprotected/proto:bare_intersection_unprotected_scenario_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "bare_intersection_unprotected_scenario_test",
    size = "small",
    srcs = ["bare_intersection_unprotected_scenario_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":bare_intersection_unprotected_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "stage_approach_test",
    size = "small",
    srcs = ["stage_approach_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":bare_intersection_unprotected_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
