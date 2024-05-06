load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "command_processor_base_lib",
    srcs = ["command_processor_base.cc"],
    hdrs = ["command_processor_base.h"],
    deps = [
        "//cyber",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/external_command/command_processor/command_processor_base/proto:command_processor_config_cc_proto",
    ],
)

# apollo_component(
#     name = "libcommand_processor_base.so",
#     deps = [":command_processor_base_lib"],
# )

apollo_cc_library(
    name = "motion_command_processor_base_lib",
    hdrs = ["motion_command_processor_base.h"],
    copts = ["-DMODULE_NAME=\\\"external_command\\\""],
    deps = [
        ":command_processor_base_lib",
        "//cyber",
        "//modules/common/util:message_util",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/external_command/command_processor/command_processor_base/proto:command_processor_config_cc_proto",
        "//modules/external_command/command_processor/command_processor_base/util:lane_way_tool",
        "//modules/external_command/command_processor/command_processor_base/util:message_reader",
        "//modules/external_command/command_processor/command_processor_base/util:message_writer",
        "//modules/routing:apollo_routing",
    ],
)

# apollo_component(
#     name = "libmotion_command_processor_base.so",
#     deps = [":motion_command_processor_base_lib"],
# )

filegroup(
    name = "conf",
    srcs = glob([
        "conf/*",
    ]),
)

cpplint()

apollo_package()
