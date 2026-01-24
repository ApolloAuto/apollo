load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "lat_based_lqr_plus_controller_conf_proto",
    srcs = ["lat_based_lqr_plus_controller_conf.proto"],
    deps=[
        "//modules/control/control_component/proto:gain_scheduler_conf_proto",
        "//modules/control/control_component/proto:leadlag_conf_proto",
        "//modules/control/control_component/proto:mrac_conf_proto",
        "//modules/control/control_task_base_extend/proto:safety_check_conf_proto",
    ],
)

apollo_package()