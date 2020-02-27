load("//tools:py_proto.bzl", "py_proto")

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
        "//modules/prediction/proto:feature_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

cc_proto_library(
    name = "chart_proto",
    deps = [
        ":chart_proto_lib",
    ],
)

proto_library(
    name = "chart_proto_lib",
    srcs = ["chart.proto"],
    deps = [
        "//modules/common/proto:geometry_proto_lib",
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
    name = "hmi_mode_proto",
    deps = [
        ":hmi_mode_proto_lib",
    ],
)

proto_library(
    name = "hmi_mode_proto_lib",
    srcs = ["hmi_mode.proto"],
    deps = [
        "//cyber/proto:scheduler_conf_proto",
    ],
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
    name = "data_collection_table_proto",
    deps = [
        ":data_collection_table_proto_lib",
    ],
)

proto_library(
    name = "data_collection_table_proto_lib",
    srcs = ["data_collection_table.proto"],
)

cc_proto_library(
    name = "camera_update_proto",
    deps = [
        ":camera_update_proto_lib",
    ],
)

proto_library(
    name = "camera_update_proto_lib",
    srcs = ["camera_update.proto"],
)

py_proto(
    name = "hmi_status_pb2",
    src = "hmi_status.proto",
    deps = [
        "//modules/common/proto:header_pb2",
        "//modules/monitor/proto:system_status_pb2",
    ],
)

py_proto(
    name = "point_cloud_pb2",
    src = "point_cloud.proto",
)

py_proto(
    name = "chart_pb2",
    src = "chart.proto",
    deps = [
        "//modules/common/proto:geometry_pb2",
    ],
)

py_proto(
    name = "hmi_mode_pb2",
    src = "hmi_mode.proto",
)

py_proto(
    name = "data_collection_table_pb2",
    src = "data_collection_table.proto",
)

py_proto(
    name = "simulation_world_pb2",
    src = "simulation_world.proto",
    deps = [
        "//modules/common/monitor_log/proto:monitor_log_pb2",
        "//modules/common/proto:pnc_point_pb2",
        "//modules/perception/proto:perception_obstacle_pb2",
        "//modules/planning/proto:planning_internal_pb2",
        "//modules/prediction/proto:feature_pb2",
        "//modules/routing/proto:routing_pb2",
    ],
)

py_proto(
    name = "camera_update_pb2",
    src = "camera_update.proto",
)

py_proto(
    name = "hmi_config_pb2",
    src = "hmi_config.proto",
)
