load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility=["//visibility:public"])

cc_library(
    name="external_command_process_component_lib",
    srcs=["external_command_process_component.cc"],
    hdrs=["external_command_process_component.h"],
    copts = ["-DMODULE_NAME=\\\"external_command_process\\\""],
    deps=[
        "//cyber",
        "//modules/external_command/process_component/proto:process_component_config_cc_proto",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/external_command/command_processor/command_processor_base:command_processor_base_lib",
        "//modules/external_command/command_processor/command_processor_base/proto:command_processor_config_cc_proto",
    ],
    alwayslink=True
)

cc_binary(
    name="libexternal_command_process_component.so",
    linkshared=True,
    linkstatic=True,
    deps=[":external_command_process_component_lib"],
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
