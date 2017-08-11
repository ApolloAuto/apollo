package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "simulation_world_proto",
    deps = [
        ":simulation_world_proto_lib",
    ],
)

proto_library(
    name = "simulation_world_proto_lib",
    srcs = ["simulation_world.proto"],
    deps = [
        "//modules/common/monitor/proto:monitor_proto_lib",
        "//modules/planning/proto:planning_proto_lib",
    ],
)
