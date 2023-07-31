load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "external_command_demo",
    hdrs = ["external_command_demo.h"],
    srcs = ["external_command_demo.cc"],
    copts = ["-DMODULE_NAME=\\\"external_command_demo\\\""],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/util:message_util",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/common_msgs/external_command_msgs:action_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:chassis_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:free_space_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:lane_follow_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:path_follow_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:speed_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:valet_parking_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/external_command/external_command_demo/proto:sweeper_custom_command_cc_proto",
        "//modules/external_command/external_command_demo/proto:demo_config_cc_proto",
    ],
)

cc_binary(
    name = "libexternal_command_demo_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":external_command_demo"],
)

cc_library(
    name = "external_command_wrapper_demo",
    hdrs = ["external_command_wrapper_demo.h"],
    srcs = ["external_command_wrapper_demo.cc"],
    copts = ["-DMODULE_NAME=\\\"external_command_demo\\\""],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common/util:message_util",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/common_msgs/external_command_msgs:action_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:chassis_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:free_space_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:lane_follow_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:path_follow_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:speed_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:valet_parking_command_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common/service_wrapper:client_wrapper",
        "//modules/external_command/external_command_demo/proto:sweeper_custom_command_cc_proto",
        "//modules/external_command/external_command_demo/proto:demo_config_cc_proto",
    ],
)

cc_binary(
    name = "libexternal_command_wrapper_demo_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":external_command_wrapper_demo"],
)

filegroup(
    name = "conf",
    srcs = glob([
        "conf/*",
        "dag/*",
        "launch/*"
    ]),
)

cpplint()
apollo_package()
