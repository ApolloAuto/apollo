load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

apollo_plugin(
    name = "liblane_follow_command_processor.so",
    srcs = ["lane_follow_command_processor.cc"],
    hdrs = ["lane_follow_command_processor.h"],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"external_command\\\""],
    deps = [
        "//cyber",
        "//modules/external_command/command_processor/command_processor_base:motion_command_processor_base_lib",
        "//modules/common_msgs/external_command_msgs:lane_follow_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
    ],
)

filegroup(
    name = "conf",
    srcs = glob([
        "conf/*",
    ]),
)

cpplint()
apollo_package()
