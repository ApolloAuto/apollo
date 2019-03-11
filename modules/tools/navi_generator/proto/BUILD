package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "trajectory_util_config_proto",
    deps = [
        ":trajectory_util_config_proto_lib",
    ],
)

proto_library(
    name = "trajectory_util_config_proto_lib",
    srcs = [
        "trajectory_util_config.proto",
    ],
    deps = [        
    ],
)

cc_proto_library(
    name = "trajectory_collector_proto",
    deps = [
        ":trajectory_collector_proto_lib",
    ],
)

proto_library(
    name = "trajectory_collector_proto_lib",
    srcs = [
        "trajectory_collector.proto",
    ],
     deps = [
    ],
)

cc_proto_library(
    name = "hmi_config_proto",
    deps = [
        ":hmi_config_proto_lib",
    ],
)

proto_library(
    name = "hmi_config_proto_lib",
    srcs = ["hmi_config.proto"],
)

cc_proto_library(
    name = "hmi_status_proto",
    deps = [
        ":hmi_status_proto_lib",
    ],
)

proto_library(
    name = "hmi_status_proto_lib",
    srcs = ["hmi_status.proto"],
    deps = [
        "//modules/common/proto:header_proto_lib",
        "//modules/monitor/proto:system_status_proto_lib",
    ],
)

cc_proto_library(
    name = "simulation_world_proto",
    deps = [
        ":simulation_world_proto_lib",
    ],
)

proto_library(
    name = "simulation_world_proto_lib",
    srcs = ["simulation_world.proto"],
)

cc_proto_library(
    name = "navigation_coordinate_proto",
    deps = [
        ":navigation_coordinate_proto_lib",
    ],
)

proto_library(
    name = "navigation_coordinate_proto_lib",
    srcs = [
        "navigation_coordinate.proto",
    ],
)

cc_proto_library(
    name = "navigation_response_proto",
    deps = [
        ":navigation_response_proto_lib",
    ],
)

proto_library(
    name = "navigation_response_proto_lib",
    srcs = [
        "navigation_response.proto",
    ],
    deps = [
         ":navigation_coordinate_proto_lib",
    ],
)

cc_proto_library(
    name = "navigation_map_data_proto",
    deps = [
        ":navigation_map_data_proto_lib",
    ],
)

proto_library(
    name = "navigation_map_data_proto_lib",
    srcs = [
        "navigation_map_data.proto",
    ],
    deps = [
         ":navigation_coordinate_proto_lib",
    ],
)
