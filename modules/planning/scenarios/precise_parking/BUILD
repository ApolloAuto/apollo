load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ]),
)

apollo_plugin(
    name = "libprecise_parking_scenario.so",
    description = ":plugins.xml",
    srcs = [
        "precise_parking_scenario.cc",
        "stage_approaching_precise_parking.cc",
        "stage_precise_parking.cc",
        "stage_arrive_parking_spot.cc",
    ],
    hdrs = [
        "precise_parking_scenario.h",
        "stage_approaching_precise_parking.h",
        "stage_precise_parking.h",
        "stage_arrive_parking_spot.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common_msgs/external_command_msgs:precise_parking_command_cc_proto",
        "//modules/planning/planning_open_space:apollo_planning_open_space",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/precise_parking/proto:precise_parking_cc_proto",
    ],
)

apollo_package(enable_source=False)

cpplint()
