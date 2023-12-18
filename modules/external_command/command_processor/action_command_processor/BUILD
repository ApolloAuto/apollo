load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")

package(default_visibility = ["//visibility:public"])

apollo_plugin(
    name = "libaction_command_processor.so",
    srcs = ["action_command_processor.cc"],
    hdrs = ["action_command_processor.h"],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"external_command\\\""],
    deps = [
        "//cyber",
        "//modules/external_command/command_processor/command_processor_base:command_processor_base_lib",
        "//modules/external_command/command_processor/command_processor_base/proto:command_processor_config_cc_proto",
        "//modules/external_command/command_processor/action_command_processor/proto:action_command_config_cc_proto",
        "//modules/external_command/command_processor/command_processor_base/util:message_reader",
        "//modules/external_command/command_processor/command_processor_base/util:message_writer",
        "//modules/common_msgs/external_command_msgs:action_command_cc_proto",
        "//modules/common_msgs/planning_msgs:pad_msg_cc_proto",
        "//modules/common_msgs/control_msgs:control_pad_msg_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/util:message_util",
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
