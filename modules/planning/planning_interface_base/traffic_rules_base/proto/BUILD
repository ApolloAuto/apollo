load("//tools:apollo_package.bzl", "apollo_package")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "traffic_rules_proto",
    srcs = ["traffic_rules_pipeline.proto"],
    deps = [
        "//modules/planning/planning_base/proto:plugin_declare_info_proto",
    ]
)

apollo_package()
