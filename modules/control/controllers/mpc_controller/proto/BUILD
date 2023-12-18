load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "mpc_controller_proto",
    srcs = ["mpc_controller.proto"],
    deps = [
        "//modules/control/control_component/proto:gain_scheduler_conf_proto",
        "//modules/control/control_component/proto:leadlag_conf_proto",
        "//modules/control/control_component/proto:pid_conf_proto",
    ],
)

apollo_package()