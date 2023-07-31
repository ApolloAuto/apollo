load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "mpc_controller_proto",
    srcs = ["mpc_controller.proto"],
    deps = [
        "//modules/control/controllers/control_task_base/proto:gain_scheduler_conf_proto",
    ],
)

apollo_package()