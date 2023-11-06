load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")

package(default_visibility=["//visibility:public"])

apollo_component(
    name="libexternal_command_process_component.so",
    srcs=["external_command_process_component.cc"],
    hdrs=["external_command_process_component.h"],
    copts = ["-DMODULE_NAME=\\\"external_command\\\""],
    deps=[
        "//cyber",
        "//modules/common/configs:config_gflags",
        "//modules/external_command/process_component/proto:process_component_config_cc_proto",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
        "//modules/external_command/command_processor/command_processor_base:command_processor_base_lib",
        "//modules/external_command/command_processor/command_processor_base/proto:command_processor_config_cc_proto",
    ],
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
