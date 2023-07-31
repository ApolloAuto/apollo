load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

EXTERNAL_COMMAND_COPTS = ['-DMODULE_NAME=\\"external_command_process\\"']

cc_library(
    name = "lane_way_tool",
    srcs = ["lane_way_tool.cc"],
    hdrs = ["lane_way_tool.h"],
    copts = EXTERNAL_COMMAND_COPTS,
    deps = [
        "//cyber",
        "//modules/map/hdmap:hdmap",
        "//modules/map/hdmap:hdmap_util",
        "//modules/common_msgs/external_command_msgs:external_geometry_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/external_command/command_processor/command_processor_base/util:message_reader",
    ],
    alwayslink = True,
)

cc_library(
    name = "message_reader",
    srcs = ["message_reader.cc"],
    hdrs = ["message_reader.h"],
    copts = EXTERNAL_COMMAND_COPTS,
    deps = [
        "//cyber",
    ],
    alwayslink = True,
)

cc_library(
    name = "message_writer",
    srcs = ["message_writer.cc"],
    hdrs = ["message_writer.h"],
    copts = EXTERNAL_COMMAND_COPTS,
    deps = [
        "//cyber",
    ],
    alwayslink = True,
)

cpplint()
apollo_package()
