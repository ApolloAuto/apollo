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
    name = "libfree_space_scenario.so",
    description = ":plugins.xml",
    srcs = [
        "free_space_scenario.cc",
        "stage_free_space.cc",
    ],
    hdrs = [
        "free_space_scenario.h",
        "stage_free_space.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common_msgs/external_command_msgs:free_space_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_open_space:apollo_planning_open_space",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/scenarios/free_space/proto:free_space_cc_proto",
    ],
)

apollo_package(enable_source=False)

cpplint()
