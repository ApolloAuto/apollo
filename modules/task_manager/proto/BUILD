load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "task_manager_proto",
    deps = [
        ":task_manager_proto_lib",
    ],
)

proto_library(
    name = "task_manager_proto_lib",
    srcs = ["task_manager.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
        "//modules/map/proto:map_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

cc_proto_library(
    name = "task_manager_config_proto",
    deps = [
        ":task_manager_config_proto_lib",
    ],
)

proto_library(
    name = "task_manager_config_proto_lib",
    srcs = ["task_manager_config.proto"],
)

cpplint()
