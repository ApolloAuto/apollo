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
        "//modules/common/monitor_log/proto:monitor_log_proto_lib",
        "//modules/common/proto:pnc_point_proto_lib",
        "//modules/perception/proto:perception_proto_lib",
        "//modules/planning/proto:planning_internal_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

cc_proto_library(
    name = "point_cloud_proto",
    deps = [
        ":point_cloud_proto_lib",
    ],
)

proto_library(
    name = "point_cloud_proto_lib",
    srcs = ["point_cloud.proto"],
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
        "//modules/monitor/proto:system_status_proto_lib",
    ],
)

cc_proto_library(
    name = "voice_detection_proto",
    deps = [
        ":voice_detection_proto_lib",
    ],
)

proto_library(
    name = "voice_detection_proto_lib",
    srcs = ["voice_detection.proto"],
)
