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
    name = "libobstacle_nudge_decider.so",
    srcs = [
        "obstacle_nudge_decider.cc",
        "nudge_calculation.cc",
    ],
    hdrs = [
        "obstacle_nudge_decider.h",
        "nudge_calculation.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    description = ":plugins.xml",
    deps = [
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/map:apollo_map",
        "//modules/planning/park_data_center:apollo_park_data_center",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/tasks/obstacle_nudge_decider/proto:obstacle_nudge_decider_cc_proto",
    ],
)

apollo_package(enable_source=False)

cpplint()
