load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility=["//visibility:public"])

cc_library(
    name="command_processor_base_lib",
    srcs=["command_processor_base.cc"],
    hdrs=["command_processor_base.h"],
    deps=[
        "//cyber",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/external_command/command_processor/command_processor_base/proto:command_processor_config_cc_proto",
    ],
    alwayslink=True
)

cc_binary(
    name="libcommand_processor_base.so",
    linkshared=True,
    linkstatic=True,
    deps=[":command_processor_base_lib"],
)

cc_library(
    name="motion_command_processor_base_lib",
    hdrs=["motion_command_processor_base.h"],
    deps=[
        "//cyber",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/external_command/command_processor/command_processor_base/proto:command_processor_config_cc_proto",
        "//modules/external_command/command_processor/command_processor_base/util:lane_way_tool",
        "//modules/external_command/command_processor/command_processor_base/util:message_reader",
        "//modules/external_command/command_processor/command_processor_base/util:message_writer",
        "//modules/routing:routing",
        "//modules/common_msgs/planning_msgs:planning_command_cc_proto",
        "//modules/common/util:message_util",
        ":command_processor_base_lib",
    ],
    alwayslink=True
)

cc_binary(
    name="libmotion_command_processor_base.so",
    linkshared=True,
    linkstatic=True,
    deps=[":motion_command_processor_base_lib"],
)

filegroup(
    name = "conf",
    srcs = glob([
        "conf/*",
    ]),
)

cpplint()
apollo_package()
