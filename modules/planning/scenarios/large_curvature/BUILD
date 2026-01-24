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
    name = "liblarge_curvature_scenario.so",
    description = ":plugins.xml",
    srcs = [
        "large_curvature_scenario.cc",
        "stage_large_curvature.cc",
    ],
    hdrs = [
        "large_curvature_scenario.h",
        "stage_large_curvature.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_open_space:apollo_planning_open_space",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/large_curvature/proto:large_curvature_cc_proto",
    ],
)

apollo_package(enable_source=False)

cpplint()
