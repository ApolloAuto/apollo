load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

filegroup(
    name = "runtime",
    srcs = glob(["conf/**"]),
)

apollo_plugin(
    name = "libspeed_setting.so",
    description = ":plugins.xml",
    srcs = ["speed_setting.cc"],
    hdrs = ["speed_setting.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/external_command_msgs:speed_command_cc_proto",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
    ],
)

apollo_package(enable_source=False)

cpplint()
