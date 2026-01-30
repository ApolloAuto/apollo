load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "antislip_control_task_conf_proto",
    srcs = ["antislip_control_task_conf.proto"],
    deps = [
        "//modules/control/control_component/proto:gain_scheduler_conf_proto",
    ],
)

apollo_package()