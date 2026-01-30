load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ])
)

apollo_plugin(
    name = "libvalet_parking_scenario_park.so",
    srcs = [
        "stage_approaching_parking_spot_park.cc",
        "stage_parking_park.cc",
        "stage_parking_retry_park.cc",
        "valet_parking_scenario.cc",
    ],
    hdrs = [
        "stage_approaching_parking_spot_park.h",
        "stage_parking_park.h",
        "stage_parking_retry_park.h",
        "valet_parking_scenario.h",
    ],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_open_space:apollo_planning_open_space",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/valet_parking_park/proto:valet_parking_park_cc_proto",
    ],
)

apollo_package(enable_source=False)
cpplint()
