load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "traffic_light_unprotected_left_turn_proto",
    srcs = ["traffic_light_unprotected_left_turn.proto"],
    deps = [
        "//modules/planning/planning_interface_base/scenario_base/proto:creep_stage_proto",
    ],
)

apollo_package()