load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_plugin(
    name = "libvalet_parking_command_processor.so",
    srcs = ["valet_parking_command_processor.cc"],
    hdrs = ["valet_parking_command_processor.h"],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"external_command\\\""],
    deps = [
        "//cyber",
        "//modules/external_command/command_processor/command_processor_base:motion_command_processor_base_lib",
        "//modules/common_msgs/external_command_msgs:valet_parking_command_cc_proto",
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
