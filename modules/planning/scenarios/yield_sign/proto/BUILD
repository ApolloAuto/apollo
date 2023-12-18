load("//tools/proto:proto.bzl", "proto_library")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "yield_sign_proto",
    srcs = ["yield_sign.proto"],
    deps = [
        "//modules/planning/planning_interface_base/scenario_base/proto:creep_stage_proto",
    ],
)

apollo_package()