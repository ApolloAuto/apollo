load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "lon_based_pid_controller_conf_proto",
    srcs = ["lon_based_pid_controller_conf.proto"],
    deps = [
        "//modules/control/controllers/control_task_base/proto:gain_scheduler_conf_proto",
        "//modules/control/controllers/control_task_base/proto:leadlag_conf_proto",
        "//modules/control/controllers/control_task_base/proto:pid_conf_proto",
    ],
)

apollo_package()