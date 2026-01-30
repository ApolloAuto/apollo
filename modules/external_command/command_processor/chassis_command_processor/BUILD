load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

apollo_plugin(
    name = "libchassis_command_processor.so",
    description = ":plugins.xml",
    srcs = ["chassis_command_processor.cc"],
    hdrs = ["chassis_command_processor.h"],
    copts = ["-DMODULE_NAME=\\\"external_command_process\\\""],
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/external_command_msgs:chassis_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/external_command/command_processor/command_processor_base:command_processor_base_lib",
        "//modules/external_command/command_processor/command_processor_base/util:message_reader",
        "//modules/external_command/command_processor/command_processor_base/util:message_writer",
    ],
)

filegroup(
    name = "conf",
    srcs = glob([
        "conf/*",
    ]),
)

cpplint()

apollo_package(enable_source = False)
