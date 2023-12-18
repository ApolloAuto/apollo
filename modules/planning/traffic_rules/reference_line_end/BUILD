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
    name = "libreference_line_end.so",
    srcs = ["reference_line_end.cc"],
    hdrs = ["reference_line_end.h"],
    description = ":plugins.xml",
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util:util_tool",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
        "//modules/planning/planning_base/proto:planning_config_cc_proto",
        "//modules/planning/traffic_rules/reference_line_end/proto:reference_line_end_cc_proto",
    ],
)

apollo_package()

cpplint()
