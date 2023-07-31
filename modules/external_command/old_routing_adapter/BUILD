load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility=["//visibility:public"])

cc_library(
    name="old_routing_adapter_lib",
    srcs=["old_routing_adapter.cc"],
    hdrs=["old_routing_adapter.h"],
    copts = [
        "-DMODULE_NAME=\\\"old_routing_adapter\\\"",
    ],
    deps=[
        "//cyber",
        "//modules/external_command/old_routing_adapter/proto:old_routing_adapter_cc_proto",
        "//modules/common_msgs/routing_msgs:routing_cc_proto",
        "//modules/map/hdmap:hdmap_util",
        "//modules/common_msgs/external_command_msgs:lane_follow_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:valet_parking_command_cc_proto",
        "//modules/common_msgs/external_command_msgs:command_status_cc_proto",
    ],
    alwayslink=True,
)

cc_binary(
    name="libold_routing_adapter.so",
    linkshared=True,
    linkstatic=True,
    deps=[":old_routing_adapter_lib"],
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