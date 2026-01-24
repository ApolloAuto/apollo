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
    name = "liblane_escape_scenario.so",
    srcs = [
        "lane_escape_scenario.cc",
        "lane_escape_stage.cc",
    ],
    hdrs = [
        "lane_escape_scenario.h",
        "lane_escape_stage.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common/status",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/park_data_center:apollo_park_data_center",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/lane_escape/proto:lane_escape_cc_proto",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

apollo_package(enable_source=False)

cpplint()
