load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_test", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libpark_and_go_scenario.so",
    srcs = [
        "park_and_go_scenario.cc",
        "stage_adjust.cc",
        "stage_check.cc",
        "stage_cruise.cc",
        "stage_pre_cruise.cc",
        "util.cc",
    ],
    hdrs = [
        "context.h",
        "park_and_go_scenario.h",
        "stage_adjust.h",
        "stage_check.h",
        "stage_cruise.h",
        "stage_pre_cruise.h",
        "util.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/park_and_go/proto:park_and_go_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_cc_test(
    name = "park_and_go_scenario_test",
    size = "small",
    srcs = ["park_and_go_scenario_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":park_and_go_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "park_and_go_stages_test",
    size = "small",
    srcs = [
        "stage_adjust_test.cc",
        "stage_check_test.cc",
        "stage_cruise_test.cc",
        "stage_pre_cruise_test.cc",
    ],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":park_and_go_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
