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
    name = "libemergency_pull_over_scenario.so",
    srcs = [
        "emergency_pull_over_scenario.cc",
        "stage_approach.cc",
        "stage_slow_down.cc",
        "stage_standby.cc",
    ],
    hdrs = [
        "emergency_pull_over_scenario.h",
        "stage_approach.h",
        "stage_slow_down.h",
        "stage_standby.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/emergency_pull_over/proto:emergency_pull_over_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "emergency_pull_over_scenario_test",
    size = "small",
    srcs = ["emergency_pull_over_scenario_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":emergency_pull_over_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "stage_slow_down_test",
    size = "small",
    srcs = ["stage_slow_down_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":emergency_pull_over_scenario_lib",
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
        ":emergency_pull_over_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "stage_standby_test",
    size = "small",
    srcs = ["stage_standby_test.cc"],
    linkopts = ["-lgomp"],
    linkstatic = True,
    deps = [
        ":emergency_pull_over_scenario_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
